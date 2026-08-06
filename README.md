# HEPTA-SAT-Flight-Software

This Arduino sketch acquires bus voltage, temperature, and nine-axis IMU data from a HEPTA-SAT V4.1.1 full board once per second. It transmits the measurements as a 30-byte binary telemetry frame that is compatible with the HEPTA Ground Station Simulator.

## Telemetry Values

- Bus voltage (V)
- Temperature (°C)
- Acceleration: AX, AY, and AZ (m/s²)
- Angular velocity: GX, GY, and GZ (deg/s)
- Magnetic field: MX, MY, and MZ (µT)

In Standby mode, the sketch acquires temperature only. In Mission mode, it acquires temperature and all nine IMU axes. Send `a` from the Ground Station to enter Mission mode or `b` to return to Standby mode.

## Ground Station Frame Format

```text
7E | Satellite ID | Type | Sequence (2 bytes, BE) | Length | Payload (23 bytes) | Checksum
```

- Satellite ID: `0x01`
- Type: Standby = `0x10`, Mission = `0x11`, Power Save = `0x12`
- Length: `0x17` (23 bytes)
- Payload: mode, voltage, temperature, acceleration XYZ, gyroscope XYZ, and magnetometer XYZ
- Voltage: ADC count = `V / (3.3 × 1.431) × 4096`
- Temperature: `°C × 10`
- Acceleration: 512 counts/g = `m/s² / 9.8 × 512`
- Angular velocity: `deg/s × 2048 / 125`
- Magnetic field: integer µT
- Checksum: one-byte sum from Satellite ID through the final payload byte; the `0x7E` header is excluded
- All two-byte integer fields use big-endian byte order

The same frame is sent through both interfaces:

1. XBee on GP15 RX and GP14 TX in Transparent mode
2. Raspberry Pi Pico 2 USB Serial for direct connection and debugging

Only binary 30-byte frames are written to the telemetry stream. No diagnostic text is mixed into the stream, preventing duplicate parsing and display overwrites in the Ground Station. Use hexadecimal display mode when inspecting the data in a terminal application.

## Implementation Notes

- Sends the Ground Station's 30-byte binary frame instead of the Lab5-03 CSV text format.
- Writes each complete frame as one binary block.
- Reports zero for all nine IMU fields in Standby mode.
- Uses the same I2C devices as the reference HEPTA implementation in Mission mode: accelerometer `0x19`, gyroscope `0x69`, and magnetometer `0x13`.
- Does not use the legacy BNO055 driver at `0x28`, because that device does not match the sensors on the target board.
- Uses the reference HEPTA voltage-divider ratio of `1.431`.
- Enters Power Save mode below 3.6 V; temperature and all IMU fields are then reported as zero.
- Does not stop when an SD card is absent.
- Explicitly enables the 3.3 V sensor power supply during startup.
- Accepts `a` and `b` commands from both XBee and Pico 2 USB Serial.

## Uploading with Arduino IDE

The Arduino sketch and its bundled library use the following Arduino-compatible directory layout:

```text
HEPTA-SAT-Flight-Software/
└── HEPTA_Flightware/
    ├── HEPTA_Flightware.ino
    └── src/
        └── HeptaSat.h
```

1. Download or clone the complete repository.
2. Open `HEPTA_Flightware/HEPTA_Flightware.ino` in Arduino IDE.
3. Select **Raspberry Pi Pico 2** as the board.
4. Select the Pico 2 USB port.
5. Click **Verify**, then **Upload**.

Do not open or upload an empty default sketch such as `sketch_aug6a.ino`.

## Uploading the Prebuilt UF2

The repository also contains `HEPTA_Flightware-Pico2.uf2`.

1. Disconnect the Pico 2 USB cable.
2. Hold the BOOTSEL button while reconnecting the USB cable.
3. Release BOOTSEL when the `RP2350` drive appears.
4. Copy `HEPTA_Flightware-Pico2.uf2` to the `RP2350` drive.

The board automatically restarts after the copy completes.

## XBee Configuration

The firmware uses 38400 baud, matching the current HEPTA XBee pair.

- `AP=0` (Transparent mode)
- Matching PAN ID on both XBee modules
- Correct `DH` and `DL` destination addresses
- `BD=5` (38400 baud)

The aircraft firmware, ground-side XBee, and Ground Station Web Serial connection must all use 38400 baud. Pico 2 USB Serial uses USB CDC, so the baud rate selected by a terminal does not change the physical USB data rate.

## Troubleshooting

If telemetry does not appear over XBee, verify the PAN ID, destination addresses, API mode, and baud rate in XCTU. For a direct USB test, select the Pico 2 USB Serial port rather than the XBee USB adapter.

The compatible Ground Station displays received hardware telemetry independently of the simulated satellite elevation.

## References

- [Lab5-03 Downlink Housekeeping Data](https://github.com/HEPTA-SAT-TRAINING/Lab5-03_downlink_hk_data)
- [HEPTA-SAT Ground Station Simulator Web](https://github.com/HEPTA-SAT-TRAINING/HEPTA-SAT-Grand-Station-Simulator-Web)
- [HEPTA-SAT Flight Software](https://github.com/hepta-sat/HEPTA-SAT-Flight-Software)
- The bundled `HEPTA_Flightware/src/` directory is based on commit `ec480993c3433461886916dbb77df1ba0d7c67bd` of [HEPTA-SAT-Library](https://github.com/HEPTA-SAT-TRAINING/HEPTA-SAT-Library).
