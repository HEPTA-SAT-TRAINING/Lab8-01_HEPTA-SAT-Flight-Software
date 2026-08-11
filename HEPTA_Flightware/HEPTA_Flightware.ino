#include "src/HeptaSat.h"

#include <math.h>
#include <Wire.h>

HeptaCdh cdh;
HeptaCom com;
HeptaEps eps;
HeptaSensor sensor;

namespace {

constexpr uint8_t TELEMETRY_HEADER = 0x7E;
constexpr uint8_t SATELLITE_ID = 0x01;
constexpr uint8_t STANDBY_PACKET_TYPE = 0x10;
constexpr uint8_t MISSION_PACKET_TYPE = 0x11;
constexpr uint8_t POWER_SAVE_PACKET_TYPE = 0x12;
constexpr uint8_t STANDBY_MODE = 0x00;
constexpr uint8_t MISSION_MODE = 0x01;

// The first 23 payload bytes are the legacy Ground Station format.  The six
// Lab5-03 EPS values are appended as millivolts/milliamps so old field offsets
// remain stable and the new values have an unambiguous wire representation.
constexpr size_t TELEMETRY_PAYLOAD_LENGTH = 35;
constexpr size_t TELEMETRY_PACKET_LENGTH = 42;
constexpr uint32_t TELEMETRY_INTERVAL_MS = 1000;
constexpr char PICTURE_FILENAME[] = "/picture_dl.jpg";

// These are the three IMU devices used by the original HEPTA-SAT Flight
// Software.  The bundled Library exposes the newer BNO055 API, so these
// board-specific devices stay behind this small adapter instead of being
// duplicated throughout the mission logic.
constexpr uint8_t ACCELEROMETER_ADDRESS = 0x19;
constexpr uint8_t GYROSCOPE_ADDRESS = 0x69;
constexpr uint8_t MAGNETOMETER_ADDRESS = 0x13;

struct TelemetryData {
  float bus_voltage_v;
  float temperature_c;
  float voltage_5v;
  float voltage_3v3;
  float solar_array_voltage_v;
  float solar_current_a;
  float bus_current_a;
  float charge_current_a;
  float acceleration[3];
  float gyro[3];
  float magnetometer[3];
};

bool mission_mode = false;
bool picture_requested = false;
uint16_t telemetry_sequence = 0;
uint16_t image_sequence = 0;
uint32_t last_telemetry_ms = 0;

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
  if (!write_i2c_register(ACCELEROMETER_ADDRESS, 0x14, 0xB6)) return false;
  delay(5);
  return write_i2c_register(ACCELEROMETER_ADDRESS, 0x0F, 0x05)
    && write_i2c_register(ACCELEROMETER_ADDRESS, 0x11, 0x00);
}

bool read_acceleration(float values[3]) {
  uint8_t data[6] = {};
  if (!read_i2c_registers(ACCELEROMETER_ADDRESS, 0x02, data, sizeof(data))) {
    return false;
  }
  for (size_t i = 0; i < 3; ++i) {
    const int16_t combined = static_cast<int16_t>(
      (static_cast<uint16_t>(data[i * 2 + 1]) << 8) | data[i * 2]);
    values[i] = static_cast<float>(combined >> 4) / 512.0f * 9.8f;
  }
  return true;
}

bool read_gyro(float values[3]) {
  if (!write_i2c_register(GYROSCOPE_ADDRESS, 0x0F, 0x04)
      || !write_i2c_register(GYROSCOPE_ADDRESS, 0x10, 0x07)
      || !write_i2c_register(GYROSCOPE_ADDRESS, 0x11, 0x00)) {
    return false;
  }
  uint8_t data[6] = {};
  for (size_t i = 0; i < sizeof(data); ++i) {
    if (!read_i2c_registers(GYROSCOPE_ADDRESS, 0x02 + i, &data[i], 1)) {
      return false;
    }
  }
  for (size_t i = 0; i < 3; ++i) {
    const int16_t combined = static_cast<int16_t>(
      (static_cast<uint16_t>(data[i * 2 + 1]) << 8) | data[i * 2]);
    values[i] = static_cast<float>(combined >> 4) * 125.0f / 2048.0f;
  }
  return true;
}

bool read_magnetometer(float values[3]) {
  if (!write_i2c_register(MAGNETOMETER_ADDRESS, 0x4B, 0x01)
      || !write_i2c_register(MAGNETOMETER_ADDRESS, 0x4C, 0x00)
      || !write_i2c_register(MAGNETOMETER_ADDRESS, 0x4E, 0x84)
      || !write_i2c_register(MAGNETOMETER_ADDRESS, 0x51, 0x04)
      || !write_i2c_register(MAGNETOMETER_ADDRESS, 0x52, 0x16)) {
    return false;
  }
  uint8_t dummy = 0;
  if (!read_i2c_registers(MAGNETOMETER_ADDRESS, 0x00, &dummy, 1)) return false;

  uint8_t data[8] = {};
  for (size_t i = 0; i < sizeof(data); ++i) {
    if (!read_i2c_registers(MAGNETOMETER_ADDRESS, 0x42 + i, &data[i], 1)) {
      return false;
    }
  }
  for (size_t i = 0; i < 3; ++i) {
    const uint16_t combined = static_cast<uint16_t>(
      (static_cast<uint16_t>(data[i * 2 + 1]) << 8) | data[i * 2]);
    int32_t raw = i == 2 ? combined >> 1 : combined >> 3;
    if (i == 2 && raw > 16383) raw -= 32768;
    if (i != 2 && raw > 4095) raw -= 8092;
    values[i] = static_cast<float>(raw);
  }
  return true;
}

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
  for (size_t i = 1; i < length_without_checksum; ++i) {
    checksum = static_cast<uint8_t>(checksum + packet[i]);
  }
  return checksum;
}

TelemetryData read_telemetry_data(float bus_voltage, bool power_save) {
  TelemetryData data = {};
  data.bus_voltage_v = bus_voltage;
  data.voltage_5v = eps.get_5v_voltage();
  data.voltage_3v3 = eps.get_3v3_voltage();
  data.solar_array_voltage_v = eps.get_sap_voltage();
  data.solar_current_a = eps.get_current_solar();
  data.bus_current_a = eps.get_current_bus();
  data.charge_current_a = eps.get_current_charge();

  if (power_save) return data;
  data.temperature_c = sensor.get_temperature();
  if (!mission_mode) return data;

  bool ok = read_acceleration(data.acceleration)
    && read_gyro(data.gyro)
    && read_magnetometer(data.magnetometer);
  if (!ok) {
    begin_reference_imu();
    delay(10);
    read_acceleration(data.acceleration);
    read_gyro(data.gyro);
    read_magnetometer(data.magnetometer);
  }
  return data;
}

size_t build_telemetry_packet(
  uint8_t *packet,
  size_t packet_capacity,
  const TelemetryData &data) {

  if (packet == nullptr || packet_capacity < TELEMETRY_PACKET_LENGTH) return 0;

  const bool power_save = !isfinite(data.bus_voltage_v) || data.bus_voltage_v < 3.6f;
  const uint8_t packet_type = power_save
    ? POWER_SAVE_PACKET_TYPE
    : mission_mode ? MISSION_PACKET_TYPE : STANDBY_PACKET_TYPE;
  const uint8_t mode = (!power_save && mission_mode) ? MISSION_MODE : STANDBY_MODE;

  size_t index = 0;
  packet[index++] = TELEMETRY_HEADER;
  packet[index++] = SATELLITE_ID;
  packet[index++] = packet_type;
  put_u16_be(packet, &index, ++telemetry_sequence);
  packet[index++] = TELEMETRY_PAYLOAD_LENGTH;
  packet[index++] = mode;

  // Legacy payload prefix.
  put_u16_be(packet, &index,
    clamp_u16(data.bus_voltage_v / (3.3f * 1.431f) * 4096.0f));
  put_i16_be(packet, &index, clamp_i16(data.temperature_c * 10.0f));
  for (float value : data.acceleration) {
    put_i16_be(packet, &index, clamp_i16(value / 9.8f * 512.0f));
  }
  for (float value : data.gyro) {
    put_i16_be(packet, &index, clamp_i16(value * 2048.0f / 125.0f));
  }
  for (float value : data.magnetometer) {
    put_i16_be(packet, &index, clamp_i16(value));
  }

  // Lab5-03 extension: millivolts then milliamps.
  put_u16_be(packet, &index, clamp_u16(data.voltage_5v * 1000.0f));
  put_u16_be(packet, &index, clamp_u16(data.voltage_3v3 * 1000.0f));
  put_u16_be(packet, &index, clamp_u16(data.solar_array_voltage_v * 1000.0f));
  put_i16_be(packet, &index, clamp_i16(data.solar_current_a * 1000.0f));
  put_i16_be(packet, &index, clamp_i16(data.bus_current_a * 1000.0f));
  put_i16_be(packet, &index, clamp_i16(data.charge_current_a * 1000.0f));

  if (index != 6 + TELEMETRY_PAYLOAD_LENGTH) return 0;
  packet[index++] = calculate_checksum(packet, index);
  return index;
}

void send_telemetry() {
  // Read the bus first because it controls the legacy power-save behavior.
  const float bus_voltage = eps.get_bus_voltage();
  const bool power_save = !isfinite(bus_voltage) || bus_voltage < 3.6f;
  const TelemetryData data = read_telemetry_data(bus_voltage, power_save);

  uint8_t packet[TELEMETRY_PACKET_LENGTH] = {};
  const size_t length = build_telemetry_packet(packet, sizeof(packet), data);
  if (length != TELEMETRY_PACKET_LENGTH) return;

  com.write(packet, length);
  // Preserve the existing direct Pico 2 USB telemetry/debug route.
  cdh.write(packet, length);
}

void apply_command(char command) {
  if (command == 'a' || command == 'A') {
    mission_mode = true;
  } else if (command == 'b' || command == 'B') {
    mission_mode = false;
  } else if (command == 'p' || command == 'P') {
    picture_requested = true;
  }
}

void process_commands() {
  while (com.is_cmd_received()) {
    apply_command(com.get_command());
  }
  while (cdh.is_cmd_received()) {
    apply_command(cdh.get_command());
  }
}

void capture_and_downlink_picture() {
  picture_requested = false;
  if (!sensor.camera_snapshot(PICTURE_FILENAME)) {
    com.send_image_error(HeptaCom::IMAGE_ERROR_CAPTURE_FAILED);
    return;
  }
  if (!com.downlink_image_file(PICTURE_FILENAME, ++image_sequence)) {
    // downlink_image_file sends a protocol ERROR when the file is unavailable.
    return;
  }
}

}  // namespace

void setup() {
  cdh.begin();
  eps.init();
  eps.switch_3V3_on();
  com.begin();
  delay(100);
  analogReadResolution(12);
  pinMode(27, INPUT);
  begin_reference_imu();
  last_telemetry_ms = millis() - TELEMETRY_INTERVAL_MS;
}

void loop() {
  process_commands();

  if (picture_requested) {
    capture_and_downlink_picture();
    last_telemetry_ms = millis();
  }

  const uint32_t now = millis();
  if (static_cast<uint32_t>(now - last_telemetry_ms) >= TELEMETRY_INTERVAL_MS) {
    last_telemetry_ms = now;
    send_telemetry();
  }

  delay(5);
}
