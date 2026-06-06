# IMDev Bot - DJI GM6020 Tank Drive

ESP32 Arduino sketch for driving two DJI RoboMaster GM6020 motors over CAN using a PS5 controller through Bluepad32.

The active sketch is:

```text
djimotor/djimotor.ino
```

## Hardware

### ESP32 Pinout

| Signal | ESP32 IO |
| --- | ---: |
| CAN_TX | IO4 |
| CAN_RX | IO5 |
| LED_G | IO13 |
| LED_B | IO14 |
| LED_R | IO15 |
| BOOT safety button | IO0 |
| Pairing switch | IO17 |

CAN runs at `1 Mbps` using the ESP32 TWAI driver.

### Motor IDs

Set the GM6020 DIP switches so the motors use these CAN IDs:

| Motor | GM6020 ID | Feedback CAN ID |
| --- | ---: | ---: |
| Left | 1 | `0x205` |
| Right | 2 | `0x206` |

Both motors are commanded through standard CAN frame `0x1FF`. The sketch maps normalized current commands from `-1.0` to `1.0` onto the GM6020 command range `-30000` to `30000`.

## Controller

Use a PS5 controller paired through Bluepad32.

Controls:

| Controller input | Behavior |
| --- | --- |
| Left stick Y | Forward/backward |
| Right stick X | Turning |
| IO17 switch | Forget Bluetooth keys and reopen pairing |
| IO0 BOOT button | Force zero motor current for 3 seconds |

Tank mix:

```text
left_current  = forward + turn
right_current = forward - turn
```

Commands are clamped to `-1.0..1.0`.

If a motor runs backward relative to the expected direction, change these constants in `djimotor.ino`:

```cpp
static constexpr float LEFT_MOTOR_SIGN = 1.0f;
static constexpr float RIGHT_MOTOR_SIGN = 1.0f;
```

Set one to `-1.0f` to reverse that side.

## Safety

The sketch forces both motor commands to zero when:

- the controller is disconnected
- IO0 BOOT is pressed
- IO17 pairing switch is pressed
- CAN is not ready

Pressing IO0 holds zero current for 3 seconds. If the controller is disconnected, zero current is held until a controller reconnects.

Status LEDs:

| LED | Meaning |
| --- | --- |
| Green, IO13 | Controller connected and drive enabled |
| Blue, IO14 | Pairing window active |
| Red, IO15 | Safety stop or disconnected |

## RTOS Tasks

| Task | Rate / behavior |
| --- | --- |
| `taskDJIMotorRead` | Async CAN receive, 100 Hz command/status cadence |
| `taskTankDriveController` | 50 Hz controller update and tank-drive command calculation |
| `taskSerialPrint` | Serial Plotter telemetry every 100 ms |

CAN receive is asynchronous: `taskDJIMotorRead` blocks on `twai_receive()` and wakes immediately when feedback arrives.

## Serial Plotter

Open Arduino IDE Serial Plotter at `115200` baud. The sketch prints tab-separated `label:value` fields, including:

```text
L_angle R_angle L_rpm R_rpm L_current R_current L_temp R_temp
L_cmd R_cmd L_online R_online rx_frames unknown_frames tx_errors
can_status can_ready controller safety pairing LY_raw RX_raw
forward turn tank_left tank_right
```

## Build

This sketch uses `Bluepad32.h`, so compile/upload with the Bluepad32 ESP32 board package, not the plain ESP32 board package.

With Arduino CLI:

```bash
arduino-cli compile --fqbn esp32-bluepad32:esp32:esp32 djimotor
```

If using Arduino IDE, select:

```text
ESP32 Dev Module from esp32-bluepad32
```

## Notes

- Use a proper CAN transceiver between the ESP32 TWAI pins and the CAN bus.
- Terminate the CAN bus correctly. Enable termination on the GM6020 only if it is physically at the end of the bus.
- The GM6020 manual included in this repo is `djimotor/gm6020_manual.pdf`.
