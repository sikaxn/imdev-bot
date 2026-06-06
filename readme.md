# IMDev Bot - DJI C620 Tank Drive

ESP32 Arduino sketch for driving four DJI C620 motor controllers over CAN using a PS5 controller through Bluepad32.

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

CAN runs at `1 Mbps` using the ESP32 TWAI driver. Use a proper CAN transceiver between the ESP32 and the CAN bus.

### C620 Motor IDs

Set the C620 controller IDs like this:

| Wheel | C620 ID | Feedback CAN ID | Command frame / slot |
| --- | ---: | ---: | --- |
| Front left | 1 | `0x201` | `0x200`, bytes 0-1 |
| Rear left | 2 | `0x202` | `0x200`, bytes 2-3 |
| Rear right | 3 | `0x203` | `0x200`, bytes 4-5 |
| Front right | 4 | `0x204` | `0x200`, bytes 6-7 |

The sketch maps normalized current commands from `-1.0` to `1.0` onto the C620 command range `-16384` to `16384`.

The M3508 motor is connected through a `19:1` gearbox to the wheel:

```text
wheel_rpm = motor_rpm / 19
```

### Mixed DJI Motor Support

The motor config includes a type per motor:

```cpp
{"FL", MOTOR_C620, 1, SIDE_LEFT, 1.0f}
```

Supported mappings:

| Type | Feedback ID | Command frames |
| --- | --- | --- |
| C620 | `0x200 + id` | IDs 1-4: `0x200`; IDs 5-8: `0x1FF` |
| GM6020 | `0x204 + id` | IDs 1-4: `0x1FF`; IDs 5-7: `0x2FF` |

C620 and GM6020 CAN addresses overlap. For example, C620 ID 5 and GM6020 ID 1 both report on `0x205`, and C620 IDs 5-8 share command frame `0x1FF` with GM6020 IDs 1-4. If both types are used later, avoid overlapping feedback IDs and command frame slots on the same CAN bus, or split them onto separate CAN buses.

## Controller

Use a PS5 controller paired through Bluepad32.

Controls:

| Controller input | Behavior |
| --- | --- |
| Left stick Y | Forward/backward |
| Right stick X | Turning |
| PS Cross/X button | Toggle direct-current mode and speed mode |
| IO17 switch | Forget Bluetooth keys and reopen pairing only when no controller is connected |
| IO0 BOOT button | Force zero motor current for 3 seconds |

Tank mix:

```text
left_current  = forward + turn
right_current = forward - turn
```

Commands are clamped to `-1.0..1.0`.

When a side command is inside deadband, the sketch applies a limited active brake based on measured motor RPM instead of simply commanding zero current. Safety stops still command true zero current.

Drive modes:

| Mode | Serial value | Behavior |
| --- | ---: | --- |
| Direct current | `mode:0` | Stick tank mix directly commands normalized motor current |
| Speed | `mode:1` | Stick tank mix becomes target wheel RPM and a PI loop commands current from motor RPM error |

Speed mode tuning constants are near the top of `djimotor.ino`. `SPEED_MAX_TARGET_RPM` is the motor-side limit; wheel target RPM is derived by dividing by the 19:1 gearbox ratio.

```cpp
static constexpr float MOTOR_TO_WHEEL_GEAR_RATIO = 19.0f;
static constexpr float SPEED_MAX_TARGET_RPM = 1000.0f;
static constexpr float BRAKE_KP = 0.0004f;
static constexpr float BRAKE_MAX_CURRENT = 0.35f;
```

If an individual motor runs backward relative to the expected direction, change its `commandSign` in `MOTOR_CONFIGS`:

```cpp
{"FL", MOTOR_C620, 1, SIDE_LEFT, -1.0f}
```

## Safety

The sketch forces all configured motor commands to zero when:

- the controller is disconnected
- IO0 BOOT is pressed
- IO17 pairing switch is pressed
- CAN is not ready

Pressing IO0 holds zero current for 3 seconds. If the controller is disconnected, zero current is held until a controller reconnects.

Status LEDs:

| LED | Meaning |
| --- | --- |
| Green, IO13 | Connected, direct-current mode |
| Green + Blue | Connected, speed mode |
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
FL_angle FL_rpm FL_wheel FL_current FL_temp FL_cmd FL_target FL_wheel_target FL_err FL_online
FR_angle FR_rpm FR_wheel FR_current FR_temp FR_cmd FR_target FR_wheel_target FR_err FR_online
RL_angle RL_rpm RL_wheel RL_current RL_temp RL_cmd RL_target RL_wheel_target RL_err RL_online
RR_angle RR_rpm RR_wheel RR_current RR_temp RR_cmd RR_target RR_wheel_target RR_err RR_online
rx_frames unknown_frames tx_errors dup_fb dup_cmd bad_cfg
can_status can_ready controller safety pairing mode LY_raw RX_raw
forward turn tank_left tank_right left_target_rpm right_target_rpm
```

`left_target_rpm` and `right_target_rpm` are wheel RPM targets. Per-motor `*_target` is motor-side target RPM, and `*_wheel_target` is wheel-side target RPM.

`dup_fb`, `dup_cmd`, and `bad_cfg` help catch future mixed C620/GM6020 configuration mistakes.

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

- Terminate the CAN bus correctly.
- C620 feedback data uses the same byte layout parsed by the sketch: angle, rpm, torque current, and temperature.
- The GM6020 manual included in this repo is `djimotor/gm6020_manual.pdf`.
