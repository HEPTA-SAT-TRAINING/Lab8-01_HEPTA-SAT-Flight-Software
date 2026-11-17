#include "src/HeptaSat.h"

#include <stdlib.h>

HeptaCdh cdh;
HeptaCom com;
HeptaEps eps;
HeptaSensor sensor;

namespace {

constexpr uint32_t DOWNLINK_INTERVAL_MS = 1000;
// ATDB is an XBee command-mode query.  Do not run it for every HK frame:
// entering/exiting command mode uses guard times and temporarily pauses the
// transparent data stream.  The last valid value is included in each HK line.
constexpr uint32_t RSSI_QUERY_INTERVAL_MS = 15000;
constexpr const char* PICTURE_FILENAME = "/picture_dl.jpg";

bool mission_mode = false;
bool picture_requested = false;
uint32_t last_downlink_ms = 0;
uint32_t last_rssi_query_ms = 0;
int rssi_dbm = 0;
bool rssi_valid = false;

bool update_xbee_rssi(void) {
  char response[24] = {};
  if (!com.enter_command_mode()) return false;

  const bool query_ok = com.send_at_command(
    "ATDB\r", response, sizeof(response), 1000);
  const bool exit_ok = com.exit_command_mode();
  if (!query_ok || !exit_ok) return false;

  // XBee ATDB returns the RSSI magnitude as a hexadecimal byte (for example,
  // "40" means -64 dBm).  Ignore ERROR/garbage rather than sending a fake
  // value to the ground station.
  char *end = nullptr;
  const unsigned long magnitude = strtoul(response, &end, 16);
  if (end == response || magnitude > 127UL) return false;

  rssi_dbm = -static_cast<int>(magnitude);
  rssi_valid = true;
  return true;
}

void refresh_xbee_rssi(void) {
  const uint32_t now = millis();
  if (static_cast<uint32_t>(now - last_rssi_query_ms) < RSSI_QUERY_INTERVAL_MS) {
    return;
  }
  last_rssi_query_ms = now;
  update_xbee_rssi();
}

// Lab5-03 wire format. Keep field names, order, precision, and units stable so
// HEPTA-SAT Serial Monitor and Ground Station can consume the same stream.
void downlink_hk_data(void) {
  const float temp = sensor.get_temperature();
  const float bus_voltage = eps.get_bus_voltage();
  const float v5 = eps.get_5v_voltage();
  const float v3v3 = eps.get_3v3_voltage();
  const float sap = eps.get_sap_voltage();
  const float isol = eps.get_current_solar();
  const float ibus = eps.get_current_bus();
  const float ichg = eps.get_current_charge();

  if (rssi_valid) {
    com.printf(
      "TEMP=%.2f,BUS=%.3f,V5=%.3f,V3V3=%.3f,SAP=%.3f,ISOL=%.3f,IBUS=%.3f,ICHG=%.3f,RSSI=%d\r\n",
      temp, bus_voltage, v5, v3v3, sap, isol, ibus, ichg, rssi_dbm);
  } else {
    com.printf(
      "TEMP=%.2f,BUS=%.3f,V5=%.3f,V3V3=%.3f,SAP=%.3f,ISOL=%.3f,IBUS=%.3f,ICHG=%.3f\r\n",
      temp, bus_voltage, v5, v3v3, sap, isol, ibus, ichg);
  }

  cdh.printf(
    "HK: TEMP=%.2f C, BUS=%.3f V, V5=%.3f V, V3V3=%.3f V, SAP=%.3f V, ISOL=%.3f A, IBUS=%.3f A, ICHG=%.3f A\r\n",
    temp, bus_voltage, v5, v3v3, sap, isol, ibus, ichg);
}

// Lab5-04 uses the same API and AX/AY/AZ field names. Mission mode extends
// that line with the remaining BNO055 gyro and magnetometer values.
void downlink_nine_axis_data(void) {
  float ax = 0.0f;
  float ay = 0.0f;
  float az = 0.0f;
  float gx = 0.0f;
  float gy = 0.0f;
  float gz = 0.0f;
  float mx = 0.0f;
  float my = 0.0f;
  float mz = 0.0f;

  const bool acceleration_ok = sensor.get_acceleration(&ax, &ay, &az);
  const bool gyro_ok = sensor.get_gyro(&gx, &gy, &gz);
  const bool magnetometer_ok = sensor.get_magnetometer(&mx, &my, &mz);

  com.printf(
    "AX=%.2f,AY=%.2f,AZ=%.2f,GX=%.2f,GY=%.2f,GZ=%.2f,MX=%.2f,MY=%.2f,MZ=%.2f\r\n",
    ax, ay, az, gx, gy, gz, mx, my, mz);

  cdh.printf(
    "IMU: AX=%.2f m/s^2, AY=%.2f m/s^2, AZ=%.2f m/s^2, GX=%.2f deg/s, GY=%.2f deg/s, GZ=%.2f deg/s, MX=%.2f uT, MY=%.2f uT, MZ=%.2f uT\r\n",
    ax, ay, az, gx, gy, gz, mx, my, mz);

  if (!acceleration_ok || !gyro_ok || !magnetometer_ok) {
    cdh.println("IMU read warning: one or more values were unavailable");
  }
}

void handle_command(char command) {
  com.printf("command = %c\r\n", command);
  cdh.printf("command = %c\r\n", command);

  if (command == 'a' || command == 'A') {
    mission_mode = true;
    com.println("MISSION_MODE");
    cdh.println("MISSION_MODE");
  } else if (command == 'b' || command == 'B') {
    mission_mode = false;
    com.println("STANDBY_MODE");
    cdh.println("STANDBY_MODE");
  } else if (command == 'p' || command == 'P') {
    picture_requested = true;
    com.println("downlink picture data");
    cdh.println("downlink picture data");
  }
}

void process_commands(void) {
  while (com.available()) {
    handle_command(com.get_char());
  }
  while (cdh.is_cmd_received()) {
    handle_command(cdh.get_command());
  }
}

void downlink_picture_data(void) {
  picture_requested = false;
  if (!sensor.camera_snapshot(PICTURE_FILENAME)) {
    com.send_image_error(HeptaCom::IMAGE_ERROR_CAPTURE_FAILED);
    cdh.println("Picture capture failed");
    return;
  }

  if (!com.downlink_image_file(PICTURE_FILENAME)) {
    cdh.printf("Picture downlink failed: %s\r\n", com.last_error());
    return;
  }
  cdh.println("Picture downlink completed");
}

}  // namespace

void setup() {
  cdh.begin();
  eps.init();
  eps.switch_3V3_on();
  sensor.begin();
  com.begin();

  cdh.println("XBee downlink started ('a'=mission, 'b'=standby, 'p'=picture)");
  last_downlink_ms = millis() - DOWNLINK_INTERVAL_MS;
  // Let the transparent link settle before the first ATDB query.
  last_rssi_query_ms = millis();
}

void loop() {
  process_commands();

  if (picture_requested) {
    // Lab5-05 sends no HK bytes while binary image packets are in flight.
    downlink_picture_data();
    last_downlink_ms = millis();
  }

  refresh_xbee_rssi();

  const uint32_t now = millis();
  if (static_cast<uint32_t>(now - last_downlink_ms) >= DOWNLINK_INTERVAL_MS) {
    last_downlink_ms = now;
    downlink_hk_data();
    if (mission_mode) downlink_nine_axis_data();
  }

  delay(5);
}
