#include "src/HeptaSat.h"

#include <math.h>
#include <SoftwareSerial.h>
#include <Wire.h>

HeptaCdh cdh;
HeptaEps eps;
HeptaSensor sensor;
SoftwareSerial xbee_serial(/*rx=*/15, /*tx=*/14);

namespace {

constexpr uint8_t TELEMETRY_HEADER = 0x7E;
constexpr uint8_t SATELLITE_ID = 0x01;
constexpr uint8_t STANDBY_PACKET_TYPE = 0x10;
constexpr uint8_t MISSION_PACKET_TYPE = 0x11;
constexpr uint8_t POWER_SAVE_PACKET_TYPE = 0x12;
constexpr uint8_t STANDBY_MODE = 0x00;
constexpr uint8_t MISSION_MODE = 0x01;

constexpr size_t TELEMETRY_PAYLOAD_LENGTH = 23;
constexpr size_t TELEMETRY_PACKET_LENGTH = 30;
constexpr uint32_t TELEMETRY_INTERVAL_MS = 1000;

// Same three sensors and I2C addresses used by HEPTA-SAT-Flight-Software.
constexpr uint8_t ACCELEROMETER_ADDRESS = 0x19;
constexpr uint8_t GYROSCOPE_ADDRESS = 0x69;
constexpr uint8_t MAGNETOMETER_ADDRESS = 0x13;

bool write_i2c_register(uint8_t address, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool read_i2c_registers(
  uint8_t address,
  uint8_t start_register,
  uint8_t *data,
  size_t length) {

  if (data == nullptr || length == 0) return false;
  Wire.beginTransmission(address);
  Wire.write(start_register);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(address, length) != length) {
    while (Wire.available()) Wire.read();
    return false;
  }
  for (size_t i = 0; i < length; ++i) {
    if (!Wire.available()) return false;
    data[i] = static_cast<uint8_t>(Wire.read());
  }
  return true;
}

bool begin_reference_imu() {
  Wire.begin();
  Wire.setClock(100000);

  // Accelerometer configuration copied from Hepta9axis::setup().
  if (!write_i2c_register(ACCELEROMETER_ADDRESS, 0x14, 0xB6)) return false;
  delay(5);
  if (!write_i2c_register(ACCELEROMETER_ADDRESS, 0x0F, 0x05)) return false;
  if (!write_i2c_register(ACCELEROMETER_ADDRESS, 0x11, 0x00)) return false;
  return true;
}

bool read_reference_acceleration(float *x, float *y, float *z) {
  uint8_t data[6] = {};
  if (!read_i2c_registers(ACCELEROMETER_ADDRESS, 0x02, data, sizeof(data))) {
    return false;
  }
  float *values[3] = {x, y, z};
  for (size_t i = 0; i < 3; ++i) {
    const int16_t combined = static_cast<int16_t>(
      (static_cast<uint16_t>(data[i * 2 + 1]) << 8) | data[i * 2]);
    const int16_t raw = combined >> 4;
    *values[i] = static_cast<float>(raw) / 512.0f * 9.8f;
  }
  return true;
}

bool read_reference_gyro(float *x, float *y, float *z) {
  if (!write_i2c_register(GYROSCOPE_ADDRESS, 0x0F, 0x04) ||
      !write_i2c_register(GYROSCOPE_ADDRESS, 0x10, 0x07) ||
      !write_i2c_register(GYROSCOPE_ADDRESS, 0x11, 0x00)) {
    return false;
  }

  uint8_t data[6] = {};
  for (size_t i = 0; i < sizeof(data); ++i) {
    if (!read_i2c_registers(
          GYROSCOPE_ADDRESS,
          static_cast<uint8_t>(0x02 + i),
          &data[i],
          1)) {
      return false;
    }
  }

  float *values[3] = {x, y, z};
  for (size_t i = 0; i < 3; ++i) {
    const int16_t combined = static_cast<int16_t>(
      (static_cast<uint16_t>(data[i * 2 + 1]) << 8) | data[i * 2]);
    const int16_t raw = combined >> 4;
    *values[i] = static_cast<float>(raw) * 125.0f / 2048.0f;
  }
  return true;
}

bool read_reference_magnetometer(float *x, float *y, float *z) {
  if (!write_i2c_register(MAGNETOMETER_ADDRESS, 0x4B, 0x01) ||
      !write_i2c_register(MAGNETOMETER_ADDRESS, 0x4C, 0x00) ||
      !write_i2c_register(MAGNETOMETER_ADDRESS, 0x4E, 0x84) ||
      !write_i2c_register(MAGNETOMETER_ADDRESS, 0x51, 0x04) ||
      !write_i2c_register(MAGNETOMETER_ADDRESS, 0x52, 0x16)) {
    return false;
  }

  // Reference Hepta9axis performs one register-zero read after power setup.
  uint8_t dummy = 0;
  if (!read_i2c_registers(MAGNETOMETER_ADDRESS, 0x00, &dummy, 1)) {
    return false;
  }

  uint8_t data[8] = {};
  for (size_t i = 0; i < sizeof(data); ++i) {
    if (!read_i2c_registers(
          MAGNETOMETER_ADDRESS,
          static_cast<uint8_t>(0x42 + i),
          &data[i],
          1)) {
      return false;
    }
  }

  float *values[3] = {x, y, z};
  for (size_t i = 0; i < 3; ++i) {
    const uint16_t combined = static_cast<uint16_t>(
      (static_cast<uint16_t>(data[i * 2 + 1]) << 8) | data[i * 2]);
    int32_t raw = i == 2 ? combined >> 1 : combined >> 3;
    if (i == 2 && raw > 16383) raw -= 32768;
    if (i != 2 && raw > 4095) raw -= 8092;
    *values[i] = static_cast<float>(raw);
  }
  return true;
}

struct SensorData {
  float temperature_c;
  float acceleration_x;
  float acceleration_y;
  float acceleration_z;
  float gyro_x;
  float gyro_y;
  float gyro_z;
  float magnetometer_x;
  float magnetometer_y;
  float magnetometer_z;
};

bool mission_mode = false;
uint16_t telemetry_sequence = 0;
uint32_t last_telemetry_ms = 0;

int16_t clamp_i16(float value) {
  if (!isfinite(value)) return 0;
  if (value > 32767.0f) return 32767;
  if (value < -32768.0f) return -32768;
  return static_cast<int16_t>(value);
}

uint16_t clamp_u16(float value) {
  if (!isfinite(value) || value < 0.0f) return 0;
  if (value > 65535.0f) return 65535;
  return static_cast<uint16_t>(value);
}

void put_u16_be(uint8_t *packet, size_t *index, uint16_t value) {
  packet[(*index)++] = static_cast<uint8_t>(value >> 8);
  packet[(*index)++] = static_cast<uint8_t>(value & 0xFF);
}

void put_i16_be(uint8_t *packet, size_t *index, int16_t value) {
  put_u16_be(packet, index, static_cast<uint16_t>(value));
}

uint8_t calculate_checksum(const uint8_t *packet, size_t length_without_checksum) {
  uint8_t checksum = 0;

  // The Ground Station checksum covers Sat ID through the end of Payload.
  // The 0x7E header itself is not included.
  for (size_t i = 1; i < length_without_checksum; ++i) {
    checksum = static_cast<uint8_t>(checksum + packet[i]);
  }

  return checksum;
}

SensorData read_sensor_data(bool read_mission_sensors, bool power_save) {
  SensorData data = {};

  if (power_save) return data;

  data.temperature_c = sensor.get_temperature();
  if (read_mission_sensors) {
    bool acceleration_ok = read_reference_acceleration(
      &data.acceleration_x,
      &data.acceleration_y,
      &data.acceleration_z);
    bool gyro_ok = read_reference_gyro(
      &data.gyro_x,
      &data.gyro_y,
      &data.gyro_z);
    bool magnetometer_ok = read_reference_magnetometer(
      &data.magnetometer_x,
      &data.magnetometer_y,
      &data.magnetometer_z);

    // Reinitialize and retry once after an I2C error or sensor power-up delay.
    if (!acceleration_ok || !gyro_ok || !magnetometer_ok) {
      begin_reference_imu();
      delay(10);
      read_reference_acceleration(
        &data.acceleration_x,
        &data.acceleration_y,
        &data.acceleration_z);
      read_reference_gyro(&data.gyro_x, &data.gyro_y, &data.gyro_z);
      read_reference_magnetometer(
        &data.magnetometer_x,
        &data.magnetometer_y,
        &data.magnetometer_z);
    }
  }

  return data;
}

size_t build_telemetry_packet(
  uint8_t *packet,
  size_t packet_capacity,
  float bus_voltage,
  const SensorData &data) {

  if (packet == nullptr || packet_capacity < TELEMETRY_PACKET_LENGTH) return 0;

  const bool power_save = bus_voltage < 3.6f;
  const uint8_t packet_type = power_save
    ? POWER_SAVE_PACKET_TYPE
    : mission_mode ? MISSION_PACKET_TYPE : STANDBY_PACKET_TYPE;
  const uint8_t mode = (!power_save && mission_mode)
    ? MISSION_MODE
    : STANDBY_MODE;

  // Match HEPTA-SAT-Flight-Software/GS/Hepta_sat_flightsoftware/main.cpp.
  const uint16_t voltage_raw =
    clamp_u16(bus_voltage / (3.3f * 1.431f) * 4096.0f);
  const int16_t temperature_raw =
    clamp_i16(data.temperature_c * 10.0f);

  const int16_t acceleration_x_raw =
    clamp_i16(data.acceleration_x / 9.8f * 512.0f);
  const int16_t acceleration_y_raw =
    clamp_i16(data.acceleration_y / 9.8f * 512.0f);
  const int16_t acceleration_z_raw =
    clamp_i16(data.acceleration_z / 9.8f * 512.0f);

  const int16_t gyro_x_raw =
    clamp_i16(data.gyro_x * 2048.0f / 125.0f);
  const int16_t gyro_y_raw =
    clamp_i16(data.gyro_y * 2048.0f / 125.0f);
  const int16_t gyro_z_raw =
    clamp_i16(data.gyro_z * 2048.0f / 125.0f);

  const int16_t magnetometer_x_raw =
    clamp_i16(data.magnetometer_x);
  const int16_t magnetometer_y_raw =
    clamp_i16(data.magnetometer_y);
  const int16_t magnetometer_z_raw =
    clamp_i16(data.magnetometer_z);

  size_t index = 0;
  packet[index++] = TELEMETRY_HEADER;
  packet[index++] = SATELLITE_ID;
  packet[index++] = packet_type;
  put_u16_be(packet, &index, ++telemetry_sequence);
  packet[index++] = TELEMETRY_PAYLOAD_LENGTH;

  packet[index++] = mode;
  put_u16_be(packet, &index, voltage_raw);
  put_i16_be(packet, &index, temperature_raw);
  put_i16_be(packet, &index, acceleration_x_raw);
  put_i16_be(packet, &index, acceleration_y_raw);
  put_i16_be(packet, &index, acceleration_z_raw);
  put_i16_be(packet, &index, gyro_x_raw);
  put_i16_be(packet, &index, gyro_y_raw);
  put_i16_be(packet, &index, gyro_z_raw);
  put_i16_be(packet, &index, magnetometer_x_raw);
  put_i16_be(packet, &index, magnetometer_y_raw);
  put_i16_be(packet, &index, magnetometer_z_raw);

  const uint8_t checksum = calculate_checksum(packet, index);
  packet[index++] = checksum;
  return index;
}

void send_telemetry() {
  // Reference HEPTA battery divider: V = ADC / 4096 * 3.3 * 1.431.
  const float bus_voltage =
    eps.get_bus_voltage_raw() / 4096.0f * 3.3f * 1.431f;
  const bool power_save = bus_voltage < 3.6f;
  const SensorData data = read_sensor_data(mission_mode, power_save);

  uint8_t packet[TELEMETRY_PACKET_LENGTH] = {};
  const size_t packet_length = build_telemetry_packet(
    packet,
    sizeof(packet),
    bus_voltage,
    data);

  if (packet_length != TELEMETRY_PACKET_LENGTH) return;

  // XBee transparent link.  Send the frame as one binary block.
  xbee_serial.write(packet, packet_length);

  // Mirror the same frame to Pico 2 USB Serial for direct connection and
  // troubleshooting without an XBee pair.
  cdh.write(packet, packet_length);

}

void apply_command(char command) {
  if (command == 'a' || command == 'A') {
    mission_mode = true;
  } else if (command == 'b' || command == 'B') {
    mission_mode = false;
  }
}

void process_commands() {
  // Commands from the XBee link.
  while (xbee_serial.available() > 0) {
    const int command = xbee_serial.read();
    if (command >= 0) apply_command(static_cast<char>(command));
  }

  // Commands from a direct Pico 2 USB connection.
  while (cdh.is_cmd_received()) {
    apply_command(cdh.get_command());
  }
}

}  // namespace

void setup() {
  cdh.begin();
  eps.init();
  eps.switch_3V3_on();
  delay(100);
  analogReadResolution(12);
  pinMode(27, INPUT);
  begin_reference_imu();
  // Match the existing HEPTA XBee pair and both Ground Station pages.
  xbee_serial.begin(38400);

  // Do not call cdh.wait_for_sd(). Telemetry must continue even when an SD
  // card is absent or failed to initialize.
  last_telemetry_ms = millis() - TELEMETRY_INTERVAL_MS;
}

void loop() {
  process_commands();

  const uint32_t now = millis();
  if (static_cast<uint32_t>(now - last_telemetry_ms) >= TELEMETRY_INTERVAL_MS) {
    last_telemetry_ms = now;
    send_telemetry();
  }

  delay(5);
}
