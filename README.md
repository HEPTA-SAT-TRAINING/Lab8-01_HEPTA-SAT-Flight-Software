# HEPTA-SAT Flight Software

Arduino Flight Software for the HEPTA-SAT V4.1.1 full board and Raspberry Pi
Pico 2. It sends housekeeping and nine-axis telemetry to the HEPTA-SAT Grand
Station Simulator Web and captures/downlinks a JPEG on command.

## Operation modes and commands

- `a`: enter Mission mode. HK, temperature, acceleration, angular velocity,
  and magnetic field are sampled once per second.
- `b`: return to Standby mode. HK and temperature continue; nine-axis values
  are zero, preserving the existing behavior.
- `p`: capture `/picture_dl.jpg` and downlink it using the Library image
  protocol. After the transfer, periodic telemetry resumes.

Below 3.6 V bus voltage, the existing Power Save behavior is retained:
temperature and nine-axis fields are zero. EPS measurements remain available
so the power condition can be diagnosed.

Commands are accepted from both XBee and Pico 2 USB Serial. Telemetry is sent
to both interfaces. JPEG downlink uses the primary XBee/`HeptaCom` path used by
Lab5-05.

## Telemetry frame

All multibyte telemetry values are big-endian.

```text
7E | Satellite ID | Type | Sequence (2) | Length | Payload (35) | Checksum
```

- Satellite ID: `0x01`
- Type: Standby `0x10`, Mission `0x11`, Power Save `0x12`
- Checksum: 8-bit sum from Satellite ID through the last payload byte. `0x7E`
  is excluded.
- Total frame size: 42 bytes

The first 23 payload bytes are unchanged from the legacy 30-byte frame:

| Offset | Value | Wire unit |
| ---: | --- | --- |
| 0 | Mode | `0` Standby / `1` Mission |
| 1-2 | Bus voltage | legacy ADC count: `V / (3.3 * 1.431) * 4096` |
| 3-4 | Temperature | signed 0.1 °C |
| 5-10 | Acceleration XYZ | signed, 512 count/g |
| 11-16 | Angular velocity XYZ | signed, 2048/125 count/(deg/s) |
| 17-22 | Magnetic field XYZ | signed integer µT |

The Lab5-03 EPS extension is appended without moving legacy fields:

| Offset | Value | Wire unit |
| ---: | --- | --- |
| 23-24 | 5 V rail | unsigned mV |
| 25-26 | 3.3 V rail | unsigned mV |
| 27-28 | Solar-array voltage | unsigned mV |
| 29-30 | Solar current | signed mA |
| 31-32 | Bus current | signed mA |
| 33-34 | Charge current | signed mA |

The physical bus voltage is read through `HeptaEps::get_bus_voltage()`. The
legacy 1.431 factor above is only the existing wire encoding and is kept so the
Grand Station conversion remains compatible.

## JPEG downlink protocol

This is the existing protocol implemented by `HeptaImageTx` and used by
Lab5-05:

```text
IMG_BEGIN\n
HP packets
\nIMG_END\n
```

Each packet contains `HP`, type, sequence, total packet count, payload length,
CRC-16/CCITT-FALSE, and payload. JPEG DATA payloads are 64 bytes. A full-image
CRC detects corruption and one XOR parity packet can recover one missing or
corrupt DATA packet. The 16-bit packet count limits this framing to 4,194,048
bytes; VGA JPEG output is expected to remain well below it. The camera writes
JPEG to `/picture_dl.jpg` on SD before downlink.

## Library usage and hardware notes

The sketch uses the bundled HEPTA-SAT Library for CDH/USB Serial, EPS setup and
measurements, XBee communication, temperature, Arducam Mini 2MP Plus capture,
SD storage, and image packetization. The bundled camera driver and sensor API
are synchronized with the referenced Library `develop` implementation. The
original Flight Software uses accelerometer `0x19`,
gyroscope `0x69`, and magnetometer `0x13`; the Library's exposed BNO055 API
targets different IMU hardware, so the three-device adapter remains in the
sketch.

XBee configuration:

- 38400 baud (`BD=5`)
- Transparent mode (`AP=0`)
- Matching PAN ID and destination addresses
- Flight Software RX GP15 / TX GP14 (provided by `HeptaCom`)

## Build and upload

Open `HEPTA_Flightware/HEPTA_Flightware.ino` in Arduino IDE, select
**Raspberry Pi Pico 2**, then Verify/Upload. The repository layout intentionally
keeps the Library under `HEPTA_Flightware/src/` so Arduino compiles it with the
sketch.

Command-line verification:

```bash
arduino-cli compile --fqbn rp2040:rp2040:rpipico2 HEPTA_Flightware
```

Do not wait indefinitely for SD at startup: telemetry must continue if the card
is absent. A `p` command reports a protocol error when capture/storage fails.

## References

- [HEPTA-SAT Library](https://github.com/HEPTA-SAT-TRAINING/HEPTA-SAT-Library)
- [Lab5-03 HK downlink](https://github.com/HEPTA-SAT-TRAINING/Lab5-03_downlink_hk_data)
- [Lab5-04 accelerometer downlink](https://github.com/HEPTA-SAT-TRAINING/Lab5-04_downlink_accelerometer_data)
- [Lab5-05 picture downlink](https://github.com/HEPTA-SAT-TRAINING/Lab5-05_downlink_picture_data)
- [HEPTA-SAT Serial Monitor](https://github.com/HEPTA-SAT-TRAINING/HEPTA-SAT-Serial_Monitor)
