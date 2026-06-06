#include <Arduino.h>
#include <Bluepad32.h>
#include <math.h>
#include "driver/twai.h"

// ESP32 TWAI/CAN pins.
static constexpr gpio_num_t CAN_TX_PIN = GPIO_NUM_4;
static constexpr gpio_num_t CAN_RX_PIN = GPIO_NUM_5;

// Status LEDs and safety/pairing buttons.
static constexpr uint8_t LED_G_PIN = 13;
static constexpr uint8_t LED_B_PIN = 14;
static constexpr uint8_t LED_R_PIN = 15;
static constexpr uint8_t BOOT_BUTTON_PIN = 0;
static constexpr uint8_t PAIR_BUTTON_PIN = 17;

enum DJIMotorType : uint8_t {
  MOTOR_C620,
  MOTOR_GM6020,
};

enum DriveSide : uint8_t {
  SIDE_NONE,
  SIDE_LEFT,
  SIDE_RIGHT,
};

enum DriveControlMode : uint8_t {
  DRIVE_DIRECT_CURRENT = 0,
  DRIVE_SPEED = 1,
};

struct DJIMotorConfig {
  const char *label;
  DJIMotorType type;
  uint8_t djiId;
  DriveSide driveSide;
  float commandSign;
};

// Current drivetrain: all four motors are C620.
// C620 feedback IDs are 0x200 + djiId. Command frames are 0x200 for IDs 1-4,
// 0x1FF for IDs 5-8. GM6020 support is kept in the mapping helpers below.
static constexpr DJIMotorConfig MOTOR_CONFIGS[] = {
    {"FL", MOTOR_C620, 1, SIDE_LEFT, 1.0f},
    {"FR", MOTOR_C620, 4, SIDE_RIGHT, 1.0f},
    {"RL", MOTOR_C620, 2, SIDE_LEFT, 1.0f},
    {"RR", MOTOR_C620, 3, SIDE_RIGHT, 1.0f},
};
static constexpr size_t MOTOR_COUNT = sizeof(MOTOR_CONFIGS) / sizeof(MOTOR_CONFIGS[0]);

static constexpr int16_t C620_MAX_COMMAND = 16384;
static constexpr int16_t GM6020_MAX_COMMAND = 30000;

static constexpr TickType_t DJIMOTOR_READ_PERIOD = pdMS_TO_TICKS(10);  // 100 Hz
static constexpr TickType_t CONTROLLER_PERIOD = pdMS_TO_TICKS(20);     // 50 Hz
static constexpr TickType_t SERIAL_PRINT_PERIOD = pdMS_TO_TICKS(100);
static constexpr uint32_t MOTOR_OFFLINE_TIMEOUT_MS = 100;
static constexpr uint32_t SAFETY_STOP_MS = 3000;
static constexpr uint32_t PAIRING_LED_MS = 5000;
static constexpr float STICK_DEADBAND = 0.06f;
static constexpr float GAMEPAD_AXIS_MAX = 512.0f;
static constexpr float MOTOR_TO_WHEEL_GEAR_RATIO = 19.0f;
static constexpr float SPEED_MAX_TARGET_RPM = 1000.0f;
static constexpr float SPEED_MAX_TARGET_WHEEL_RPM =
    SPEED_MAX_TARGET_RPM / MOTOR_TO_WHEEL_GEAR_RATIO;
static constexpr float SPEED_KP = 0.1f;
static constexpr float SPEED_KI = 0.1f;
static constexpr float SPEED_INTEGRAL_LIMIT = 5000.0f;
static constexpr float SPEED_CONTROL_DT = 0.020f;
static constexpr float BRAKE_RPM_DEADBAND = 20.0f;
static constexpr float BRAKE_KP = 0.0004f;
static constexpr float BRAKE_MAX_CURRENT = 0.35f;

struct DJIMotorRuntime {
  uint32_t feedbackId = 0;
  uint32_t commandId = 0;
  uint8_t commandSlot = 0;
  uint16_t rawAngle = 0;
  float angleDeg = 0.0f;
  int16_t rpm = 0;
  int16_t torqueCurrent = 0;
  uint8_t temperatureC = 0;
  uint32_t lastUpdateMs = 0;
  uint32_t frameCount = 0;
  int16_t commandRaw = 0;
  float commandNormalized = 0.0f;
  float targetRpm = 0.0f;
  float targetWheelRpm = 0.0f;
  float speedErrorRpm = 0.0f;
  float speedIntegral = 0.0f;
  bool online = false;
};

static DJIMotorRuntime motors[MOTOR_COUNT];

static uint32_t rxFrameCount = 0;
static uint32_t unknownFrameCount = 0;
static uint32_t txErrorCount = 0;
static uint32_t duplicateFeedbackIdCount = 0;
static uint32_t duplicateCommandSlotCount = 0;
static uint32_t invalidMotorConfigCount = 0;
static esp_err_t canInitStatus = ESP_OK;
static bool canReady = false;

static portMUX_TYPE motorMux = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE controlMux = portMUX_INITIALIZER_UNLOCKED;

static GamepadPtr controller = nullptr;
static bool controllerConnected = false;
static bool controllerSafetyStop = true;
static bool pairingMode = false;
static DriveControlMode driveControlMode = DRIVE_DIRECT_CURRENT;
static uint32_t safetyStopUntilMs = 0;
static uint32_t pairingLedUntilMs = 0;
static int16_t leftStickYRaw = 0;
static int16_t rightStickXRaw = 0;
static float driveForward = 0.0f;
static float driveTurn = 0.0f;
static float driveLeftCurrent = 0.0f;
static float driveRightCurrent = 0.0f;
static float driveLeftTargetRpm = 0.0f;
static float driveRightTargetRpm = 0.0f;

static uint16_t readU16BE(const uint8_t *data) {
  return (static_cast<uint16_t>(data[0]) << 8) | data[1];
}

static int16_t readI16BE(const uint8_t *data) {
  return static_cast<int16_t>(readU16BE(data));
}

static void writeI16BE(uint8_t *data, int16_t value) {
  data[0] = static_cast<uint8_t>((value >> 8) & 0xFF);
  data[1] = static_cast<uint8_t>(value & 0xFF);
}

static bool timeBefore(uint32_t nowMs, uint32_t deadlineMs) {
  return static_cast<int32_t>(deadlineMs - nowMs) > 0;
}

static float clampFloat(float value, float minValue, float maxValue) {
  if (value > maxValue) {
    return maxValue;
  }
  if (value < minValue) {
    return minValue;
  }
  return value;
}

static float normalizeStickAxis(int16_t rawValue) {
  float value = clampFloat(static_cast<float>(rawValue) / GAMEPAD_AXIS_MAX, -1.0f, 1.0f);
  const float absValue = fabsf(value);
  if (absValue < STICK_DEADBAND) {
    return 0.0f;
  }

  const float scaled = (absValue - STICK_DEADBAND) / (1.0f - STICK_DEADBAND);
  return (value > 0.0f) ? scaled : -scaled;
}

static float brakeCurrentForMotorRpm(float motorRpm) {
  if (fabsf(motorRpm) < BRAKE_RPM_DEADBAND) {
    return 0.0f;
  }
  return clampFloat(-motorRpm * BRAKE_KP, -BRAKE_MAX_CURRENT, BRAKE_MAX_CURRENT);
}

static int16_t maxCommandForMotor(DJIMotorType type) {
  switch (type) {
    case MOTOR_C620:
      return C620_MAX_COMMAND;
    case MOTOR_GM6020:
      return GM6020_MAX_COMMAND;
    default:
      return 0;
  }
}

static int16_t normalizedToCommand(DJIMotorType type, float normalizedCurrent) {
  const int16_t maxCommand = maxCommandForMotor(type);
  normalizedCurrent = clampFloat(normalizedCurrent, -1.0f, 1.0f);
  return static_cast<int16_t>(lroundf(normalizedCurrent * maxCommand));
}

static float commandToNormalized(DJIMotorType type, int16_t command) {
  const int16_t maxCommand = maxCommandForMotor(type);
  if (maxCommand == 0) {
    return 0.0f;
  }
  return static_cast<float>(command) / static_cast<float>(maxCommand);
}

static uint32_t feedbackIdForMotor(DJIMotorType type, uint8_t djiId) {
  switch (type) {
    case MOTOR_C620:
      return 0x200 + djiId;
    case MOTOR_GM6020:
      return 0x204 + djiId;
    default:
      return 0;
  }
}

static uint32_t commandIdForMotor(DJIMotorType type, uint8_t djiId) {
  switch (type) {
    case MOTOR_C620:
      return (djiId <= 4) ? 0x200 : 0x1FF;
    case MOTOR_GM6020:
      return (djiId <= 4) ? 0x1FF : 0x2FF;
    default:
      return 0;
  }
}

static uint8_t commandSlotForMotor(uint8_t djiId) {
  return (djiId - 1) % 4;
}

static bool isValidMotorConfig(const DJIMotorConfig &config) {
  if (config.djiId == 0) {
    return false;
  }

  switch (config.type) {
    case MOTOR_C620:
      return config.djiId <= 8;
    case MOTOR_GM6020:
      return config.djiId <= 7;
    default:
      return false;
  }
}

static void setStatusLight(bool green, bool blue, bool red) {
  digitalWrite(LED_G_PIN, green ? HIGH : LOW);
  digitalWrite(LED_B_PIN, blue ? HIGH : LOW);
  digitalWrite(LED_R_PIN, red ? HIGH : LOW);
}

static void updateStatusLight(uint32_t nowMs, bool connected, bool timedSafetyStop) {
  const bool pairingActive = timeBefore(nowMs, pairingLedUntilMs);
  DriveControlMode modeSnapshot;

  portENTER_CRITICAL(&controlMux);
  modeSnapshot = driveControlMode;
  pairingMode = pairingActive;
  portEXIT_CRITICAL(&controlMux);

  if (pairingActive) {
    setStatusLight(false, true, false);
  } else if (timedSafetyStop) {
    setStatusLight(false, false, true);
  } else if (connected) {
    setStatusLight(true, modeSnapshot == DRIVE_SPEED, false);
  } else {
    setStatusLight(false, false, true);
  }
}

static void updateDriveState(float forward, float turn, float leftCurrent, float rightCurrent,
                             float leftTargetRpm, float rightTargetRpm, int16_t leftYRaw,
                             int16_t rightXRaw, bool connected, bool safetyStop) {
  portENTER_CRITICAL(&controlMux);
  driveForward = forward;
  driveTurn = turn;
  driveLeftCurrent = leftCurrent;
  driveRightCurrent = rightCurrent;
  driveLeftTargetRpm = leftTargetRpm;
  driveRightTargetRpm = rightTargetRpm;
  leftStickYRaw = leftYRaw;
  rightStickXRaw = rightXRaw;
  controllerConnected = connected;
  controllerSafetyStop = safetyStop;
  portEXIT_CRITICAL(&controlMux);
}

static void zeroDriveState(bool connected, bool safetyStop) {
  updateDriveState(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0, 0, connected, safetyStop);
}

static void setConfiguredMotorCurrent(size_t motorIndex, float current) {
  if (motorIndex >= MOTOR_COUNT) {
    return;
  }

  const DJIMotorConfig &config = MOTOR_CONFIGS[motorIndex];
  const int16_t command = normalizedToCommand(config.type, current);

  portENTER_CRITICAL(&motorMux);
  motors[motorIndex].commandRaw = command;
  motors[motorIndex].commandNormalized = commandToNormalized(config.type, command);
  motors[motorIndex].targetRpm = 0.0f;
  motors[motorIndex].targetWheelRpm = 0.0f;
  motors[motorIndex].speedErrorRpm = 0.0f;
  motors[motorIndex].speedIntegral = 0.0f;
  portEXIT_CRITICAL(&motorMux);
}

static void resetSpeedControllerState() {
  portENTER_CRITICAL(&motorMux);
  for (size_t i = 0; i < MOTOR_COUNT; i++) {
    motors[i].targetRpm = 0.0f;
    motors[i].targetWheelRpm = 0.0f;
    motors[i].speedErrorRpm = 0.0f;
    motors[i].speedIntegral = 0.0f;
  }
  portEXIT_CRITICAL(&motorMux);
}

static void runTankSpeedController(float leftSpeedCommand, float rightSpeedCommand,
                                   float &leftCurrentOut, float &rightCurrentOut) {
  float leftCurrentSum = 0.0f;
  float rightCurrentSum = 0.0f;
  uint8_t leftCount = 0;
  uint8_t rightCount = 0;

  portENTER_CRITICAL(&motorMux);
  for (size_t i = 0; i < MOTOR_COUNT; i++) {
    const DJIMotorConfig &config = MOTOR_CONFIGS[i];
    float sideCommand = 0.0f;
    if (config.driveSide == SIDE_LEFT) {
      sideCommand = leftSpeedCommand;
    } else if (config.driveSide == SIDE_RIGHT) {
      sideCommand = rightSpeedCommand;
    }

    const float targetWheelRpm = sideCommand * SPEED_MAX_TARGET_WHEEL_RPM;
    const float targetMotorRpm = targetWheelRpm * MOTOR_TO_WHEEL_GEAR_RATIO * config.commandSign;
    motors[i].targetWheelRpm = targetWheelRpm;
    motors[i].targetRpm = targetMotorRpm;

    float currentCommand = 0.0f;
    if (!motors[i].online) {
      motors[i].speedErrorRpm = 0.0f;
      motors[i].speedIntegral = 0.0f;
    } else if (fabsf(targetMotorRpm) < 1.0f) {
      motors[i].speedErrorRpm = -static_cast<float>(motors[i].rpm);
      motors[i].speedIntegral = 0.0f;
      currentCommand = brakeCurrentForMotorRpm(static_cast<float>(motors[i].rpm));
    } else {
      const float error = targetMotorRpm - static_cast<float>(motors[i].rpm);
      motors[i].speedErrorRpm = error;
      motors[i].speedIntegral += error * SPEED_CONTROL_DT;
      motors[i].speedIntegral =
          clampFloat(motors[i].speedIntegral, -SPEED_INTEGRAL_LIMIT, SPEED_INTEGRAL_LIMIT);
      currentCommand = SPEED_KP * error + SPEED_KI * motors[i].speedIntegral;
      currentCommand = clampFloat(currentCommand, -1.0f, 1.0f);
    }

    const int16_t commandRaw = normalizedToCommand(config.type, currentCommand);
    motors[i].commandRaw = commandRaw;
    motors[i].commandNormalized = commandToNormalized(config.type, commandRaw);

    const float physicalCurrent = motors[i].commandNormalized * config.commandSign;
    if (config.driveSide == SIDE_LEFT) {
      leftCurrentSum += physicalCurrent;
      leftCount++;
    } else if (config.driveSide == SIDE_RIGHT) {
      rightCurrentSum += physicalCurrent;
      rightCount++;
    }
  }
  portEXIT_CRITICAL(&motorMux);

  leftCurrentOut = (leftCount > 0) ? leftCurrentSum / leftCount : 0.0f;
  rightCurrentOut = (rightCount > 0) ? rightCurrentSum / rightCount : 0.0f;
}

static void runTankDirectCurrentController(float leftCommand, float rightCommand,
                                           float &leftCurrentOut, float &rightCurrentOut) {
  float leftCurrentSum = 0.0f;
  float rightCurrentSum = 0.0f;
  uint8_t leftCount = 0;
  uint8_t rightCount = 0;

  portENTER_CRITICAL(&motorMux);
  for (size_t i = 0; i < MOTOR_COUNT; i++) {
    const DJIMotorConfig &config = MOTOR_CONFIGS[i];
    float sideCommand = 0.0f;
    if (config.driveSide == SIDE_LEFT) {
      sideCommand = leftCommand;
    } else if (config.driveSide == SIDE_RIGHT) {
      sideCommand = rightCommand;
    }

    float motorCurrent = sideCommand * config.commandSign;
    if (fabsf(sideCommand) < 0.001f && motors[i].online) {
      motorCurrent = brakeCurrentForMotorRpm(static_cast<float>(motors[i].rpm));
    }

    const int16_t commandRaw = normalizedToCommand(config.type, motorCurrent);
    motors[i].commandRaw = commandRaw;
    motors[i].commandNormalized = commandToNormalized(config.type, commandRaw);
    motors[i].targetRpm = 0.0f;
    motors[i].targetWheelRpm = 0.0f;
    motors[i].speedErrorRpm = (fabsf(sideCommand) < 0.001f) ? -static_cast<float>(motors[i].rpm)
                                                            : 0.0f;
    motors[i].speedIntegral = 0.0f;

    const float physicalCurrent = motors[i].commandNormalized * config.commandSign;
    if (config.driveSide == SIDE_LEFT) {
      leftCurrentSum += physicalCurrent;
      leftCount++;
    } else if (config.driveSide == SIDE_RIGHT) {
      rightCurrentSum += physicalCurrent;
      rightCount++;
    }
  }
  portEXIT_CRITICAL(&motorMux);

  leftCurrentOut = (leftCount > 0) ? leftCurrentSum / leftCount : 0.0f;
  rightCurrentOut = (rightCount > 0) ? rightCurrentSum / rightCount : 0.0f;
}

// Public API for future expansion. Type is required because C620 and GM6020 CAN
// IDs overlap on shared buses.
void setMotorCurrent(DJIMotorType type, uint8_t djiId, float current) {
  for (size_t i = 0; i < MOTOR_COUNT; i++) {
    if (MOTOR_CONFIGS[i].type == type && MOTOR_CONFIGS[i].djiId == djiId) {
      setConfiguredMotorCurrent(i, current);
    }
  }
}

void setC620MotorCurrent(uint8_t djiId, float current) {
  setMotorCurrent(MOTOR_C620, djiId, current);
}

void setGM6020MotorCurrent(uint8_t djiId, float current) {
  setMotorCurrent(MOTOR_GM6020, djiId, current);
}

void setTankCurrents(float leftCurrent, float rightCurrent) {
  for (size_t i = 0; i < MOTOR_COUNT; i++) {
    float current = 0.0f;
    if (MOTOR_CONFIGS[i].driveSide == SIDE_LEFT) {
      current = leftCurrent;
    } else if (MOTOR_CONFIGS[i].driveSide == SIDE_RIGHT) {
      current = rightCurrent;
    }

    setConfiguredMotorCurrent(i, current * MOTOR_CONFIGS[i].commandSign);
  }
}

void stopMotors() {
  setTankCurrents(0.0f, 0.0f);
}

static bool controllerConnectedSnapshot() {
  bool connected;
  portENTER_CRITICAL(&controlMux);
  connected = controllerConnected;
  portEXIT_CRITICAL(&controlMux);
  return connected;
}

static void enterSafetyStop(uint32_t nowMs) {
  safetyStopUntilMs = nowMs + SAFETY_STOP_MS;
  stopMotors();
  zeroDriveState(controllerConnectedSnapshot(), true);
}

void onConnectedGamepad(GamepadPtr gp) {
  portENTER_CRITICAL(&controlMux);
  if (controller == nullptr) {
    controller = gp;
  }
  controllerConnected = true;
  controllerSafetyStop = false;
  portEXIT_CRITICAL(&controlMux);
}

void onDisconnectedGamepad(GamepadPtr gp) {
  portENTER_CRITICAL(&controlMux);
  if (controller == gp) {
    controller = nullptr;
  }
  portEXIT_CRITICAL(&controlMux);

  stopMotors();
  zeroDriveState(false, true);
}

static void initMotorState() {
  portENTER_CRITICAL(&motorMux);
  duplicateFeedbackIdCount = 0;
  duplicateCommandSlotCount = 0;
  invalidMotorConfigCount = 0;

  for (size_t i = 0; i < MOTOR_COUNT; i++) {
    const DJIMotorConfig &config = MOTOR_CONFIGS[i];
    if (!isValidMotorConfig(config)) {
      invalidMotorConfigCount++;
    }

    motors[i] = DJIMotorRuntime{};
    motors[i].feedbackId = feedbackIdForMotor(config.type, config.djiId);
    motors[i].commandId = commandIdForMotor(config.type, config.djiId);
    motors[i].commandSlot = commandSlotForMotor(config.djiId);

    for (size_t j = 0; j < i; j++) {
      if (motors[j].feedbackId == motors[i].feedbackId) {
        duplicateFeedbackIdCount++;
      }
      if (motors[j].commandId == motors[i].commandId &&
          motors[j].commandSlot == motors[i].commandSlot) {
        duplicateCommandSlotCount++;
      }
    }
  }
  portEXIT_CRITICAL(&motorMux);
}

static bool initCAN() {
  twai_general_config_t gConfig =
      TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  gConfig.rx_queue_len = 64;
  gConfig.tx_queue_len = 8;

  twai_timing_config_t tConfig = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t fConfig = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  canInitStatus = twai_driver_install(&gConfig, &tConfig, &fConfig);
  if (canInitStatus != ESP_OK) {
    canReady = false;
    return false;
  }

  canInitStatus = twai_start();
  canReady = canInitStatus == ESP_OK;
  return canReady;
}

static void updateMotorState(DJIMotorRuntime &motor, const twai_message_t &msg, uint32_t nowMs) {
  motor.rawAngle = readU16BE(&msg.data[0]);
  motor.angleDeg = static_cast<float>(motor.rawAngle) * (360.0f / 8192.0f);
  motor.rpm = readI16BE(&msg.data[2]);
  motor.torqueCurrent = readI16BE(&msg.data[4]);
  motor.temperatureC = msg.data[6];
  motor.lastUpdateMs = nowMs;
  motor.frameCount++;
  motor.online = true;
}

static void handleCANFrame(const twai_message_t &msg) {
  if (msg.extd || msg.rtr || msg.data_length_code < 7) {
    return;
  }

  const uint32_t nowMs = millis();
  bool matched = false;

  portENTER_CRITICAL(&motorMux);
  rxFrameCount++;
  for (size_t i = 0; i < MOTOR_COUNT; i++) {
    if (msg.identifier == motors[i].feedbackId) {
      updateMotorState(motors[i], msg, nowMs);
      matched = true;
    }
  }
  if (!matched) {
    unknownFrameCount++;
  }
  portEXIT_CRITICAL(&motorMux);
}

static void refreshMotorOnlineStatus(uint32_t nowMs) {
  portENTER_CRITICAL(&motorMux);
  for (size_t i = 0; i < MOTOR_COUNT; i++) {
    motors[i].online =
        motors[i].lastUpdateMs != 0 && (nowMs - motors[i].lastUpdateMs) <= MOTOR_OFFLINE_TIMEOUT_MS;
  }
  portEXIT_CRITICAL(&motorMux);
}

static void sendDJIMotorCommands() {
  if (!canReady) {
    return;
  }

  struct CommandFrame {
    uint32_t identifier = 0;
    int16_t command[4] = {0, 0, 0, 0};
    bool active = false;
  };

  CommandFrame frames[MOTOR_COUNT];

  portENTER_CRITICAL(&motorMux);
  for (size_t i = 0; i < MOTOR_COUNT; i++) {
    size_t frameIndex = MOTOR_COUNT;
    for (size_t frame = 0; frame < MOTOR_COUNT; frame++) {
      if (frames[frame].active && frames[frame].identifier == motors[i].commandId) {
        frameIndex = frame;
        break;
      }
    }

    if (frameIndex == MOTOR_COUNT) {
      for (size_t frame = 0; frame < MOTOR_COUNT; frame++) {
        if (!frames[frame].active) {
          frames[frame].active = true;
          frames[frame].identifier = motors[i].commandId;
          frameIndex = frame;
          break;
        }
      }
    }

    if (frameIndex < MOTOR_COUNT && motors[i].commandSlot < 4) {
      frames[frameIndex].command[motors[i].commandSlot] = motors[i].commandRaw;
    }
  }
  portEXIT_CRITICAL(&motorMux);

  for (size_t frame = 0; frame < MOTOR_COUNT; frame++) {
    if (!frames[frame].active) {
      continue;
    }

    twai_message_t msg = {};
    msg.identifier = frames[frame].identifier;
    msg.extd = 0;
    msg.rtr = 0;
    msg.data_length_code = 8;

    writeI16BE(&msg.data[0], frames[frame].command[0]);
    writeI16BE(&msg.data[2], frames[frame].command[1]);
    writeI16BE(&msg.data[4], frames[frame].command[2]);
    writeI16BE(&msg.data[6], frames[frame].command[3]);

    const esp_err_t txStatus = twai_transmit(&msg, pdMS_TO_TICKS(2));
    if (txStatus != ESP_OK) {
      portENTER_CRITICAL(&motorMux);
      txErrorCount++;
      portEXIT_CRITICAL(&motorMux);
    }
  }
}

void taskDJIMotorRead(void *pvParameters) {
  (void)pvParameters;

  TickType_t lastControlTick = xTaskGetTickCount();
  for (;;) {
    if (!canReady) {
      vTaskDelay(DJIMOTOR_READ_PERIOD);
      lastControlTick = xTaskGetTickCount();
      continue;
    }

    const TickType_t nowTick = xTaskGetTickCount();
    if ((nowTick - lastControlTick) >= DJIMOTOR_READ_PERIOD) {
      lastControlTick = nowTick;
      refreshMotorOnlineStatus(millis());
      sendDJIMotorCommands();
    }

    const TickType_t elapsedTicks = xTaskGetTickCount() - lastControlTick;
    const TickType_t waitTicks =
        (elapsedTicks < DJIMOTOR_READ_PERIOD) ? (DJIMOTOR_READ_PERIOD - elapsedTicks) : 0;

    twai_message_t msg;
    if (twai_receive(&msg, waitTicks) == ESP_OK) {
      handleCANFrame(msg);
    }
  }
}

void taskTankDriveController(void *pvParameters) {
  (void)pvParameters;

  bool lastPairButtonPressed = false;
  bool lastModeTogglePressed = false;
  for (;;) {
    BP32.update();

    const uint32_t nowMs = millis();
    const bool bootPressed = digitalRead(BOOT_BUTTON_PIN) == LOW;
    const bool pairButtonPressed = digitalRead(PAIR_BUTTON_PIN) == LOW;

    GamepadPtr gp;
    portENTER_CRITICAL(&controlMux);
    gp = controller;
    portEXIT_CRITICAL(&controlMux);

    const bool connected = gp != nullptr && gp->isConnected();
    const bool timedSafetyStop = timeBefore(nowMs, safetyStopUntilMs);

    if (bootPressed) {
      enterSafetyStop(nowMs);
    }

    if (!connected && pairButtonPressed && !lastPairButtonPressed) {
      pairingLedUntilMs = nowMs + PAIRING_LED_MS;
      enterSafetyStop(nowMs);
      portENTER_CRITICAL(&controlMux);
      controller = nullptr;
      portEXIT_CRITICAL(&controlMux);
      BP32.forgetBluetoothKeys();
      BP32.enableNewBluetoothConnections(true);
    }
    lastPairButtonPressed = pairButtonPressed;

    if (!connected) {
      lastModeTogglePressed = false;
      stopMotors();
      zeroDriveState(false, true);
      updateStatusLight(nowMs, false, false);
      vTaskDelay(CONTROLLER_PERIOD);
      continue;
    }

    // Bluepad32 maps the PS Cross/X button to a().
    const bool modeTogglePressed = gp->a();
    if (modeTogglePressed && !lastModeTogglePressed) {
      portENTER_CRITICAL(&controlMux);
      driveControlMode =
          (driveControlMode == DRIVE_DIRECT_CURRENT) ? DRIVE_SPEED : DRIVE_DIRECT_CURRENT;
      portEXIT_CRITICAL(&controlMux);
      resetSpeedControllerState();
      stopMotors();
    }
    lastModeTogglePressed = modeTogglePressed;

    if (!canReady || timedSafetyStop) {
      stopMotors();
      zeroDriveState(true, true);
      updateStatusLight(nowMs, true, true);
      vTaskDelay(CONTROLLER_PERIOD);
      continue;
    }

    const int16_t leftYRaw = gp->axisY();
    const int16_t rightXRaw = gp->axisRX();
    const float forward = normalizeStickAxis(-leftYRaw);
    const float turn = normalizeStickAxis(rightXRaw);
    const float leftCommand = clampFloat(forward + turn, -1.0f, 1.0f);
    const float rightCommand = clampFloat(forward - turn, -1.0f, 1.0f);

    DriveControlMode modeSnapshot;
    portENTER_CRITICAL(&controlMux);
    modeSnapshot = driveControlMode;
    portEXIT_CRITICAL(&controlMux);

    float leftCurrent = leftCommand;
    float rightCurrent = rightCommand;
    float leftTargetRpm = 0.0f;
    float rightTargetRpm = 0.0f;
    if (modeSnapshot == DRIVE_SPEED) {
      leftTargetRpm = leftCommand * SPEED_MAX_TARGET_WHEEL_RPM;
      rightTargetRpm = rightCommand * SPEED_MAX_TARGET_WHEEL_RPM;
      runTankSpeedController(leftCommand, rightCommand, leftCurrent, rightCurrent);
    } else {
      runTankDirectCurrentController(leftCommand, rightCommand, leftCurrent, rightCurrent);
    }

    updateDriveState(forward, turn, leftCurrent, rightCurrent, leftTargetRpm, rightTargetRpm,
                     leftYRaw, rightXRaw, true, false);
    updateStatusLight(nowMs, true, false);

    vTaskDelay(CONTROLLER_PERIOD);
  }
}

void taskSerialPrint(void *pvParameters) {
  (void)pvParameters;

  TickType_t lastWake = xTaskGetTickCount();
  for (;;) {
    DJIMotorRuntime motorSnapshot[MOTOR_COUNT];
    uint32_t rxFrames;
    uint32_t unknownFrames;
    uint32_t txErrors;
    uint32_t duplicateFeedbackIds;
    uint32_t duplicateCommandSlots;
    uint32_t invalidMotorConfigs;
    esp_err_t initStatus;
    bool canReadySnapshot;
    bool controllerConnectedSnapshotValue;
    bool controllerSafetyStopSnapshot;
    bool pairingModeSnapshot;
    DriveControlMode driveControlModeSnapshot;
    int16_t leftStickYRawSnapshot;
    int16_t rightStickXRawSnapshot;
    float driveForwardSnapshot;
    float driveTurnSnapshot;
    float driveLeftCurrentSnapshot;
    float driveRightCurrentSnapshot;
    float driveLeftTargetRpmSnapshot;
    float driveRightTargetRpmSnapshot;

    portENTER_CRITICAL(&motorMux);
    for (size_t i = 0; i < MOTOR_COUNT; i++) {
      motorSnapshot[i] = motors[i];
    }
    rxFrames = rxFrameCount;
    unknownFrames = unknownFrameCount;
    txErrors = txErrorCount;
    duplicateFeedbackIds = duplicateFeedbackIdCount;
    duplicateCommandSlots = duplicateCommandSlotCount;
    invalidMotorConfigs = invalidMotorConfigCount;
    initStatus = canInitStatus;
    canReadySnapshot = canReady;
    portEXIT_CRITICAL(&motorMux);

    portENTER_CRITICAL(&controlMux);
    controllerConnectedSnapshotValue = controllerConnected;
    controllerSafetyStopSnapshot = controllerSafetyStop;
    pairingModeSnapshot = pairingMode;
    driveControlModeSnapshot = driveControlMode;
    leftStickYRawSnapshot = leftStickYRaw;
    rightStickXRawSnapshot = rightStickXRaw;
    driveForwardSnapshot = driveForward;
    driveTurnSnapshot = driveTurn;
    driveLeftCurrentSnapshot = driveLeftCurrent;
    driveRightCurrentSnapshot = driveRightCurrent;
    driveLeftTargetRpmSnapshot = driveLeftTargetRpm;
    driveRightTargetRpmSnapshot = driveRightTargetRpm;
    portEXIT_CRITICAL(&controlMux);

    for (size_t i = 0; i < MOTOR_COUNT; i++) {
      Serial.print(MOTOR_CONFIGS[i].label);
      Serial.print("_angle:");
      Serial.print(motorSnapshot[i].angleDeg, 2);
      Serial.print('\t');
      Serial.print(MOTOR_CONFIGS[i].label);
      Serial.print("_rpm:");
      Serial.print(motorSnapshot[i].rpm);
      Serial.print('\t');
      Serial.print(MOTOR_CONFIGS[i].label);
      Serial.print("_wheel:");
      Serial.print(static_cast<float>(motorSnapshot[i].rpm) * MOTOR_CONFIGS[i].commandSign /
                       MOTOR_TO_WHEEL_GEAR_RATIO,
                   1);
      Serial.print('\t');
      Serial.print(MOTOR_CONFIGS[i].label);
      Serial.print("_current:");
      Serial.print(motorSnapshot[i].torqueCurrent);
      Serial.print('\t');
      Serial.print(MOTOR_CONFIGS[i].label);
      Serial.print("_temp:");
      Serial.print(motorSnapshot[i].temperatureC);
      Serial.print('\t');
      Serial.print(MOTOR_CONFIGS[i].label);
      Serial.print("_cmd:");
      Serial.print(motorSnapshot[i].commandNormalized, 3);
      Serial.print('\t');
      Serial.print(MOTOR_CONFIGS[i].label);
      Serial.print("_target:");
      Serial.print(motorSnapshot[i].targetRpm, 0);
      Serial.print('\t');
      Serial.print(MOTOR_CONFIGS[i].label);
      Serial.print("_wheel_target:");
      Serial.print(motorSnapshot[i].targetWheelRpm, 1);
      Serial.print('\t');
      Serial.print(MOTOR_CONFIGS[i].label);
      Serial.print("_err:");
      Serial.print(motorSnapshot[i].speedErrorRpm, 0);
      Serial.print('\t');
      Serial.print(MOTOR_CONFIGS[i].label);
      Serial.print("_online:");
      Serial.print(motorSnapshot[i].online ? 1 : 0);
      Serial.print('\t');
    }

    Serial.print("rx_frames:");
    Serial.print(rxFrames);
    Serial.print('\t');
    Serial.print("unknown_frames:");
    Serial.print(unknownFrames);
    Serial.print('\t');
    Serial.print("tx_errors:");
    Serial.print(txErrors);
    Serial.print('\t');
    Serial.print("dup_fb:");
    Serial.print(duplicateFeedbackIds);
    Serial.print('\t');
    Serial.print("dup_cmd:");
    Serial.print(duplicateCommandSlots);
    Serial.print('\t');
    Serial.print("bad_cfg:");
    Serial.print(invalidMotorConfigs);
    Serial.print('\t');
    Serial.print("can_status:");
    Serial.print(static_cast<int>(initStatus));
    Serial.print('\t');
    Serial.print("can_ready:");
    Serial.print(canReadySnapshot ? 1 : 0);
    Serial.print('\t');
    Serial.print("controller:");
    Serial.print(controllerConnectedSnapshotValue ? 1 : 0);
    Serial.print('\t');
    Serial.print("safety:");
    Serial.print(controllerSafetyStopSnapshot ? 1 : 0);
    Serial.print('\t');
    Serial.print("pairing:");
    Serial.print(pairingModeSnapshot ? 1 : 0);
    Serial.print('\t');
    Serial.print("mode:");
    Serial.print(static_cast<int>(driveControlModeSnapshot));
    Serial.print('\t');
    Serial.print("LY_raw:");
    Serial.print(leftStickYRawSnapshot);
    Serial.print('\t');
    Serial.print("RX_raw:");
    Serial.print(rightStickXRawSnapshot);
    Serial.print('\t');
    Serial.print("forward:");
    Serial.print(driveForwardSnapshot, 3);
    Serial.print('\t');
    Serial.print("turn:");
    Serial.print(driveTurnSnapshot, 3);
    Serial.print('\t');
    Serial.print("tank_left:");
    Serial.print(driveLeftCurrentSnapshot, 3);
    Serial.print('\t');
    Serial.print("tank_right:");
    Serial.print(driveRightCurrentSnapshot, 3);
    Serial.print('\t');
    Serial.print("left_target_rpm:");
    Serial.print(driveLeftTargetRpmSnapshot, 0);
    Serial.print('\t');
    Serial.print("right_target_rpm:");
    Serial.println(driveRightTargetRpmSnapshot, 0);

    vTaskDelayUntil(&lastWake, SERIAL_PRINT_PERIOD);
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(LED_G_PIN, OUTPUT);
  pinMode(LED_B_PIN, OUTPUT);
  pinMode(LED_R_PIN, OUTPUT);
  pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);
  pinMode(PAIR_BUTTON_PIN, INPUT_PULLUP);
  setStatusLight(false, false, true);

  BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);
  BP32.enableNewBluetoothConnections(true);

  initMotorState();
  stopMotors();
  initCAN();

  xTaskCreatePinnedToCore(taskDJIMotorRead, "taskDJIMotorRead", 4096, nullptr, 3, nullptr, 0);
  xTaskCreatePinnedToCore(taskTankDriveController, "taskTankDriveController", 4096, nullptr, 2,
                          nullptr, 1);
  xTaskCreatePinnedToCore(taskSerialPrint, "taskSerialPrint", 4096, nullptr, 1, nullptr, 1);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
