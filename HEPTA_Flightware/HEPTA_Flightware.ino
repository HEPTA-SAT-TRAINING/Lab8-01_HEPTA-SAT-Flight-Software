#include "src/HeptaSat.h"

#include <math.h>

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

// The first 23 payload bytes retain the legacy Ground Station layout. The six
// Lab5-03 EPS values are appended in millivolts/milliamps.
constexpr size_t TELEMETRY_PAYLOAD_LENGTH = 35;
constexpr size_t TELEMETRY_PACKET_LENGTH = 42;
constexpr uint32_t TELEMETRY_INTERVAL_MS = 1000;
constexpr char PICTURE_FILENAME[] = "/picture_dl.jpg";

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

  // Lab5-03 HK acquisition through the public HEPTA-SAT-Library API.
  data.voltage_5v = eps.get_5v_voltage();
  data.voltage_3v3 = eps.get_3v3_voltage();
  data.solar_array_voltage_v = eps.get_sap_voltage();
  data.solar_current_a = eps.get_current_solar();
  data.bus_current_a = eps.get_current_bus();
  data.charge_current_a = eps.get_current_charge();

  if (power_save) return data;

  data.temperature_c = sensor.get_temperature();
  if (!mission_mode) return data;

  // The Library owns sensor addresses, initialization, conversions, retry,
  // and pin assignments. Failed reads remain zero without stopping telemetry.
  sensor.get_acceleration(
    &data.acceleration[0], &data.acceleration[1], &data.acceleration[2]);
  sensor.get_gyro(&data.gyro[0], &data.gyro[1], &data.gyro[2]);
  sensor.get_magnetometer(
    &data.magnetometer[0], &data.magnetometer[1], &data.magnetometer[2]);
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

  // Legacy 23-byte payload prefix.
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

  // Lab5-03 extension: millivolts followed by milliamps.
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
  const float bus_voltage = eps.get_bus_voltage();
  const bool power_save = !isfinite(bus_voltage) || bus_voltage < 3.6f;
  const TelemetryData data = read_telemetry_data(bus_voltage, power_save);

  uint8_t packet[TELEMETRY_PACKET_LENGTH] = {};
  const size_t length = build_telemetry_packet(packet, sizeof(packet), data);
  if (length != TELEMETRY_PACKET_LENGTH) return;

  com.write(packet, length);
  // Preserve direct Pico 2 USB telemetry for bench testing.
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
  com.downlink_image_file(PICTURE_FILENAME, ++image_sequence);
}

}  // namespace

void setup() {
  cdh.begin();
  eps.init();
  eps.switch_3V3_on();
  sensor.begin();
  com.begin();
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
