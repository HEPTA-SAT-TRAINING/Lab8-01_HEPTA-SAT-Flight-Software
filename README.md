# HEPTA-SAT Flight Software

Flight Software for the HEPTA-SAT V4.1.1 full board and Raspberry Pi Pico 2.
The sketch follows the actual Lab5-03, Lab5-04, and Lab5-05 command and
downlink formats and uses the public APIs exposed by `HeptaSat.h`.

## Commands and operation

Commands are single characters without a required line ending:

- `a`: enter Mission mode. HK and nine-axis data are downlinked every second.
- `b`: return to Standby mode. HK data continues and nine-axis downlink stops.
- `p`: capture `/picture_dl.jpg` and downlink it with the Library image
  protocol. Periodic text telemetry resumes after transfer.

Commands received over XBee are read with `HeptaCom::available()` and
`get_char()`. The same commands are accepted from Pico 2 USB Serial through
`HeptaCdh::is_cmd_received()` and `get_command()` for bench testing.

## Text telemetry format

`downlink_hk_data()` emits the exact Lab5-03 field names, order, precision, and
CRLF termination through `com.printf()`:

```text
TEMP=23.40,BUS=3.780,V5=5.001,V3V3=3.301,SAP=4.120,ISOL=0.104,IBUS=0.217,ICHG=0.031
```

The corresponding `cdh.printf()` line includes labels and units for the USB
debug console. Units on the XBee line are:

| Field | Measurement | Unit |
| --- | --- | --- |
| `TEMP` | temperature | °C |
| `BUS` | bus voltage | V |
| `V5` | 5 V rail | V |
| `V3V3` | 3.3 V rail | V |
| `SAP` | solar-array voltage | V |
| `ISOL` | solar current | A |
| `IBUS` | bus current | A |
| `ICHG` | charge current | A |

Mission mode extends the Lab5-04 `AX,AY,AZ` line with the remaining BNO055
nine-axis values:

```text
AX=0.01,AY=-0.03,AZ=9.80,GX=0.10,GY=0.20,GZ=-0.10,MX=21.00,MY=-4.00,MZ=38.00
```

Acceleration is in m/s², gyro is in deg/s, and magnetometer is in µT. Sensor
addresses, initialization, pin assignments, and conversions belong to the
Library and are not duplicated in the sketch.

## JPEG downlink

The `p` flow matches Lab5-05:

1. `sensor.camera_snapshot("/picture_dl.jpg")`
2. On failure, `com.send_image_error(IMAGE_ERROR_CAPTURE_FAILED)`
3. On success, `com.downlink_image_file("/picture_dl.jpg")`

`downlink_image_file()` produces the Library protocol consumed by
HEPTA-SAT Serial Monitor and Grand Station Simulator Web:

```text
IMG_BEGIN
HP START/DATA/PARITY/END packets
IMG_END
```

Periodic HK/IMU text is not inserted into the binary image transfer.

## Library dependency

`HEPTA_Flightware/src` is a Git submodule of
[HEPTA-SAT-Library](https://github.com/HEPTA-SAT-TRAINING/HEPTA-SAT-Library).
It is pinned to the commit used by the current Lab5-05 picture example:

```text
b5266a05b16414d43891ac3c11dfdb3c3491865d
```

This commit contains the RP2040 Arducam JPEG capture fixes and resolution API.
The Library source is neither copied nor modified in this repository.

Clone and initialize the fixed dependency:

```bash
git clone --recurse-submodules \
  https://github.com/HEPTA-SAT-TRAINING/HEPTA-SAT-Flight-Software.git
cd HEPTA-SAT-Flight-Software
git submodule status
```

For an existing clone or after switching branches:

```bash
git submodule sync --recursive
git submodule update --init --recursive
git submodule status
```

Do not use `git submodule update --remote` for a normal build; the parent
gitlink is the reproducible dependency version.

## Build

Open `HEPTA_Flightware/HEPTA_Flightware.ino` in Arduino IDE, select
**Raspberry Pi Pico 2**, then Verify/Upload. Command-line verification:

```bash
arduino-cli compile --fqbn rp2040:rp2040:rpipico2 HEPTA_Flightware
```

## Reference implementations

- [Lab5-03 HK downlink](https://github.com/HEPTA-SAT-TRAINING/Lab5-03_downlink_hk_data)
- [Lab5-04 accelerometer downlink](https://github.com/HEPTA-SAT-TRAINING/Lab5-04_downlink_accelerometer_data)
- [Lab5-05 picture downlink](https://github.com/HEPTA-SAT-TRAINING/Lab5-05_downlink_picture_data)
- [HEPTA-SAT Serial Monitor](https://github.com/HEPTA-SAT-TRAINING/HEPTA-SAT-Serial_Monitor)
