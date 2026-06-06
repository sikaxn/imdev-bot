# DJI Motor CAN Protocol Reference

This note documents the CAN mappings used by `djimotor.ino` for DJI C620 and GM6020 devices.

CAN bus settings:

| Setting | Value |
| --- | --- |
| Bit rate | `1 Mbps` |
| Frame format | Standard 11-bit CAN ID |
| Payload length | 8 bytes |
| Byte order | Big-endian for 16-bit values |

## Current Drivetrain

All drive motors are C620 controllers:

| Wheel | Type | DJI ID | Feedback ID | Command frame | Command bytes |
| --- | --- | ---: | ---: | ---: | --- |
| Front left | C620 | 1 | `0x201` | `0x200` | 0-1 |
| Rear left | C620 | 2 | `0x202` | `0x200` | 2-3 |
| Rear right | C620 | 3 | `0x203` | `0x200` | 4-5 |
| Front right | C620 | 4 | `0x204` | `0x200` | 6-7 |

## Feedback Frames

The feedback frame byte layout is the same shape for C620 and GM6020.

| Bytes | Field | Type | Notes |
| --- | --- | --- | --- |
| 0-1 | Rotor angle | `uint16_t` | Raw encoder count, `0..8191` |
| 2-3 | Speed | `int16_t` | RPM |
| 4-5 | Torque/current feedback | `int16_t` | Device feedback value |
| 6 | Temperature | `uint8_t` | Celsius |
| 7 | Reserved / unused | `uint8_t` | Not used by this sketch |

Angle conversion:

```text
angle_degrees = raw_angle * 360.0 / 8192.0
```

The current robot uses M3508 motors through a `19:1` gearbox to the wheels. The CAN feedback speed is motor-side RPM, so wheel speed is:

```text
wheel_rpm = motor_rpm / 19
motor_rpm = wheel_rpm * 19
```

### C620 Feedback IDs

C620 feedback ID is:

```text
feedback_id = 0x200 + dji_id
```

| C620 ID | Feedback ID |
| ---: | ---: |
| 1 | `0x201` |
| 2 | `0x202` |
| 3 | `0x203` |
| 4 | `0x204` |
| 5 | `0x205` |
| 6 | `0x206` |
| 7 | `0x207` |
| 8 | `0x208` |

### GM6020 Feedback IDs

GM6020 feedback ID is:

```text
feedback_id = 0x204 + dji_id
```

| GM6020 ID | Feedback ID |
| ---: | ---: |
| 1 | `0x205` |
| 2 | `0x206` |
| 3 | `0x207` |
| 4 | `0x208` |
| 5 | `0x209` |
| 6 | `0x20A` |
| 7 | `0x20B` |

## Command Frames

Each command frame carries four signed 16-bit command values.

| Bytes | Slot | Motor ID mapping |
| --- | ---: | --- |
| 0-1 | 0 | First motor in command group |
| 2-3 | 1 | Second motor in command group |
| 4-5 | 2 | Third motor in command group |
| 6-7 | 3 | Fourth motor in command group |

The sketch calculates the slot as:

```text
slot = (dji_id - 1) % 4
```

The sketch writes each command as big-endian `int16_t`.

### C620 Commands

C620 command values control current output.

| C620 ID range | Command CAN ID | Command range |
| --- | ---: | ---: |
| 1-4 | `0x200` | `-16384..16384` |
| 5-8 | `0x1FF` | `-16384..16384` |

For C620 IDs 1-4:

| C620 ID | Frame | Bytes |
| ---: | ---: | --- |
| 1 | `0x200` | 0-1 |
| 2 | `0x200` | 2-3 |
| 3 | `0x200` | 4-5 |
| 4 | `0x200` | 6-7 |

For C620 IDs 5-8:

| C620 ID | Frame | Bytes |
| ---: | ---: | --- |
| 5 | `0x1FF` | 0-1 |
| 6 | `0x1FF` | 2-3 |
| 7 | `0x1FF` | 4-5 |
| 8 | `0x1FF` | 6-7 |

### GM6020 Commands

GM6020 command values control voltage output in the DJI manual. The sketch still exposes a normalized `-1.0..1.0` API for consistency.

| GM6020 ID range | Command CAN ID | Command range |
| --- | ---: | ---: |
| 1-4 | `0x1FF` | `-30000..30000` |
| 5-7 | `0x2FF` | `-30000..30000` |

For GM6020 IDs 1-4:

| GM6020 ID | Frame | Bytes |
| ---: | ---: | --- |
| 1 | `0x1FF` | 0-1 |
| 2 | `0x1FF` | 2-3 |
| 3 | `0x1FF` | 4-5 |
| 4 | `0x1FF` | 6-7 |

For GM6020 IDs 5-7:

| GM6020 ID | Frame | Bytes |
| ---: | ---: | --- |
| 5 | `0x2FF` | 0-1 |
| 6 | `0x2FF` | 2-3 |
| 7 | `0x2FF` | 4-5 |

## C620 / GM6020 Overlap

C620 and GM6020 addresses overlap. Do not place overlapping devices on the same CAN bus unless the software is intentionally designed to handle the ambiguity.

### Feedback ID Overlap

| CAN ID | C620 device | GM6020 device |
| ---: | --- | --- |
| `0x205` | C620 ID 5 | GM6020 ID 1 |
| `0x206` | C620 ID 6 | GM6020 ID 2 |
| `0x207` | C620 ID 7 | GM6020 ID 3 |
| `0x208` | C620 ID 8 | GM6020 ID 4 |

If one feedback CAN ID is shared, one received frame cannot identify which physical motor sent it.

### Command Frame Overlap

The command frame `0x1FF` is shared:

| Frame / bytes | C620 target | GM6020 target |
| --- | --- | --- |
| `0x1FF`, bytes 0-1 | C620 ID 5 | GM6020 ID 1 |
| `0x1FF`, bytes 2-3 | C620 ID 6 | GM6020 ID 2 |
| `0x1FF`, bytes 4-5 | C620 ID 7 | GM6020 ID 3 |
| `0x1FF`, bytes 6-7 | C620 ID 8 | GM6020 ID 4 |

If both devices listen to the same frame and slot, one command controls both.

## Safe Expansion Rules

Use these rules when adding motors:

- C620 IDs 1-4 are safe with GM6020 IDs 1-7 from a command-frame perspective, but C620 ID 4 feedback `0x204` does not overlap GM6020 feedback.
- Avoid C620 IDs 5-8 when GM6020 IDs 1-4 are on the same bus.
- Avoid duplicate `feedback_id` values.
- Avoid duplicate `(command_id, slot)` pairs.
- If overlap is unavoidable, split the motor groups onto separate CAN buses.

`djimotor.ino` reports these diagnostics in Serial Plotter output:

| Field | Meaning |
| --- | --- |
| `dup_fb` | Duplicate feedback ID found in `MOTOR_CONFIGS` |
| `dup_cmd` | Duplicate command frame and slot found in `MOTOR_CONFIGS` |
| `bad_cfg` | Invalid motor type or ID found in `MOTOR_CONFIGS` |

## Source References

- DJI C620 user guide: `https://cdn-hz.robomaster.com/robomasters/public/document/RoboMaster%20C620%20Brushless%20DC%20Motor%20Speed%20Controller%20V1.0.pdf`
- DJI GM6020 user guide: `gm6020_manual.pdf` in this folder, and DJI-hosted copy at `https://rm-static.djicdn.com/tem/17348/RoboMaster%20GM6020%20Brushless%20DC%20Motor%20User%20Guide.pdf`
