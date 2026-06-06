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

// GM6020 CAN protocol values.
static constexpr uint32_t GM6020_CONTROL_ID_1_TO_4 = 0x1FF;
static constexpr uint32_t GM6020_FEEDBACK_BASE_ID = 0x204;
static constexpr uint8_t LEFT_MOTOR_ID = 1;
static constexpr uint8_t RIGHT_MOTOR_ID = 2;
static constexpr int16_t GM6020_MAX_COMMAND = 30000;

static constexpr TickType_t DJIMOTOR_READ_PERIOD = pdMS_TO_TICKS(10);     // 100 Hz
static constexpr TickType_t CONTROLLER_PERIOD = pdMS_TO_TICKS(20);         // 50 Hz
static constexpr TickType_t SERIAL_PRINT_PERIOD = pdMS_TO_TICKS(100);
static constexpr uint32_t MOTOR_OFFLINE_TIMEOUT_MS = 100;
static constexpr uint32_t SAFETY_STOP_MS = 3000;
static constexpr uint32_t PAIRING_LED_MS = 5000;
static constexpr float STICK_DEADBAND = 0.06f;
static constexpr float GAMEPAD_AXIS_MAX = 512.0f;
static constexpr float LEFT_MOTOR_SIGN = 1.0f;
static constexpr float RIGHT_MOTOR_SIGN = 1.0f;

struct DJIMotorState {
  uint8_t motorId = 0;
  uint32_t feedbackId = 0;
  uint16_t rawAngle = 0;
  float angleDeg = 0.0f;
  int16_t rpm = 0;
  int16_t torqueCurrent = 0;
  uint8_t temperatureC = 0;
  uint32_t lastUpdateMs = 0;
  uint32_t frameCount = 0;
  bool online = false;
};

static DJIMotorState leftMotor;
static DJIMotorState rightMotor;

static int16_t motorCommandRaw[4] = {0, 0, 0, 0};
static uint32_t rxFrameCount = 0;
static uint32_t unknownFrameCount = 0;
static uint32_t txErrorCount = 0;
static esp_err_t canInitStatus = ESP_OK;
static bool canReady = false;

static portMUX_TYPE motorMux = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE controlMux = portMUX_INITIALIZER_UNLOCKED;

static GamepadPtr controller = nullptr;
static bool controllerConnected = false;
static bool controllerSafetyStop = true;
static bool pairingMode = false;
static uint32_t safetyStopUntilMs = 0;
static uint32_t pairingLedUntilMs = 0;
static int16_t leftStickYRaw = 0;
static int16_t rightStickXRaw = 0;
static float driveForward = 0.0f;
static float driveTurn = 0.0f;
static float driveLeftCurrent = 0.0f;
static float driveRightCurrent = 0.0f;

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

static int16_t normalizedToCommand(float normalizedCurrent) {
  if (normalizedCurrent > 1.0f) {
    normalizedCurrent = 1.0f;
  } else if (normalizedCurrent < -1.0f) {
    normalizedCurrent = -1.0f;
  }

  return static_cast<int16_t>(lroundf(normalizedCurrent * GM6020_MAX_COMMAND));
}

static float commandToNormalized(int16_t command) {
  return static_cast<float>(command) / static_cast<float>(GM6020_MAX_COMMAND);
}

static uint32_t feedbackIdForMotor(uint8_t motorId) {
  return GM6020_FEEDBACK_BASE_ID + motorId;
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

static void setStatusLight(bool green, bool blue, bool red) {
  digitalWrite(LED_G_PIN, green ? HIGH : LOW);
  digitalWrite(LED_B_PIN, blue ? HIGH : LOW);
  digitalWrite(LED_R_PIN, red ? HIGH : LOW);
}

static void updateStatusLight(uint32_t nowMs, bool connected, bool timedSafetyStop) {
  const bool pairingActive = timeBefore(nowMs, pairingLedUntilMs);
  if (timedSafetyStop) {
    setStatusLight(false, false, true);
  } else if (pairingActive) {
    setStatusLight(false, true, false);
  } else if (connected) {
    setStatusLight(true, false, false);
  } else {
    setStatusLight(false, false, true);
  }

  portENTER_CRITICAL(&controlMux);
  pairingMode = pairingActive;
  portEXIT_CRITICAL(&controlMux);
}

static void updateDriveState(float forward, float turn, float leftCurrent, float rightCurrent,
                             int16_t leftYRaw, int16_t rightXRaw, bool connected,
                             bool safetyStop) {
  portENTER_CRITICAL(&controlMux);
  driveForward = forward;
  driveTurn = turn;
  driveLeftCurrent = leftCurrent;
  driveRightCurrent = rightCurrent;
  leftStickYRaw = leftYRaw;
  rightStickXRaw = rightXRaw;
  controllerConnected = connected;
  controllerSafetyStop = safetyStop;
  portEXIT_CRITICAL(&controlMux);
}

static void zeroDriveState(bool connected, bool safetyStop) {
  updateDriveState(0.0f, 0.0f, 0.0f, 0.0f, 0, 0, connected, safetyStop);
}

// Public API: current is normalized to -1.0 .. 1.0, where 0.0 stops the motor.
void setMotorCurrent(uint8_t motorId, float current) {
  if (motorId < 1 || motorId > 4) {
    return;
  }

  const int16_t command = normalizedToCommand(current);
  portENTER_CRITICAL(&motorMux);
  motorCommandRaw[motorId - 1] = command;
  portEXIT_CRITICAL(&motorMux);
}

void setMotorCurrents(float leftCurrent, float rightCurrent) {
  const int16_t leftCommand = normalizedToCommand(leftCurrent);
  const int16_t rightCommand = normalizedToCommand(rightCurrent);

  portENTER_CRITICAL(&motorMux);
  motorCommandRaw[LEFT_MOTOR_ID - 1] = leftCommand;
  motorCommandRaw[RIGHT_MOTOR_ID - 1] = rightCommand;
  portEXIT_CRITICAL(&motorMux);
}

void setLeftMotorCurrent(float current) {
  setMotorCurrent(LEFT_MOTOR_ID, current);
}

void setRightMotorCurrent(float current) {
  setMotorCurrent(RIGHT_MOTOR_ID, current);
}

void stopMotors() {
  setMotorCurrents(0.0f, 0.0f);
}

static void enterSafetyStop(uint32_t nowMs) {
  safetyStopUntilMs = nowMs + SAFETY_STOP_MS;
  stopMotors();
  zeroDriveState(controllerConnected, true);
}

void onConnectedGamepad(GamepadPtr gp) {
  if (controller == nullptr) {
    controller = gp;
  }

  portENTER_CRITICAL(&controlMux);
  controllerConnected = true;
  controllerSafetyStop = false;
  portEXIT_CRITICAL(&controlMux);
}

void onDisconnectedGamepad(GamepadPtr gp) {
  if (controller == gp) {
    controller = nullptr;
  }

  stopMotors();
  zeroDriveState(false, true);
}

static void initMotorState() {
  portENTER_CRITICAL(&motorMux);
  leftMotor.motorId = LEFT_MOTOR_ID;
  leftMotor.feedbackId = feedbackIdForMotor(LEFT_MOTOR_ID);
  rightMotor.motorId = RIGHT_MOTOR_ID;
  rightMotor.feedbackId = feedbackIdForMotor(RIGHT_MOTOR_ID);
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

static void updateMotorState(DJIMotorState &motor, const twai_message_t &msg, uint32_t nowMs) {
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

  portENTER_CRITICAL(&motorMux);
  rxFrameCount++;
  if (msg.identifier == leftMotor.feedbackId) {
    updateMotorState(leftMotor, msg, nowMs);
  } else if (msg.identifier == rightMotor.feedbackId) {
    updateMotorState(rightMotor, msg, nowMs);
  } else {
    unknownFrameCount++;
  }
  portEXIT_CRITICAL(&motorMux);
}

static void refreshMotorOnlineStatus(uint32_t nowMs) {
  portENTER_CRITICAL(&motorMux);
  leftMotor.online =
      leftMotor.lastUpdateMs != 0 && (nowMs - leftMotor.lastUpdateMs) <= MOTOR_OFFLINE_TIMEOUT_MS;
  rightMotor.online =
      rightMotor.lastUpdateMs != 0 && (nowMs - rightMotor.lastUpdateMs) <= MOTOR_OFFLINE_TIMEOUT_MS;
  portEXIT_CRITICAL(&motorMux);
}

static void sendGM6020Command() {
  if (!canReady) {
    return;
  }

  int16_t commands[4];

  portENTER_CRITICAL(&motorMux);
  for (uint8_t i = 0; i < 4; i++) {
    commands[i] = motorCommandRaw[i];
  }
  portEXIT_CRITICAL(&motorMux);

  twai_message_t msg = {};
  msg.identifier = GM6020_CONTROL_ID_1_TO_4;
  msg.extd = 0;
  msg.rtr = 0;
  msg.data_length_code = 8;

  writeI16BE(&msg.data[0], commands[0]);
  writeI16BE(&msg.data[2], commands[1]);
  writeI16BE(&msg.data[4], commands[2]);
  writeI16BE(&msg.data[6], commands[3]);

  const esp_err_t txStatus = twai_transmit(&msg, pdMS_TO_TICKS(2));
  if (txStatus != ESP_OK) {
    portENTER_CRITICAL(&motorMux);
    txErrorCount++;
    portEXIT_CRITICAL(&motorMux);
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
      sendGM6020Command();
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
  for (;;) {
    BP32.update();

    const uint32_t nowMs = millis();
    const bool bootPressed = digitalRead(BOOT_BUTTON_PIN) == LOW;
    const bool pairButtonPressed = digitalRead(PAIR_BUTTON_PIN) == LOW;

    if (bootPressed) {
      enterSafetyStop(nowMs);
    }

    if (pairButtonPressed && !lastPairButtonPressed) {
      pairingLedUntilMs = nowMs + PAIRING_LED_MS;
      enterSafetyStop(nowMs);
      controller = nullptr;
      BP32.forgetBluetoothKeys();
      BP32.enableNewBluetoothConnections(true);
    }
    lastPairButtonPressed = pairButtonPressed;

    GamepadPtr gp = controller;
    const bool connected = gp != nullptr && gp->isConnected();
    const bool timedSafetyStop = timeBefore(nowMs, safetyStopUntilMs);

    if (!connected) {
      stopMotors();
      zeroDriveState(false, true);
      updateStatusLight(nowMs, false, false);
      vTaskDelay(CONTROLLER_PERIOD);
      continue;
    }

    if (timedSafetyStop) {
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
    const float leftCurrent = clampFloat(forward + turn, -1.0f, 1.0f) * LEFT_MOTOR_SIGN;
    const float rightCurrent = clampFloat(forward - turn, -1.0f, 1.0f) * RIGHT_MOTOR_SIGN;

    setMotorCurrents(leftCurrent, rightCurrent);
    updateDriveState(forward, turn, leftCurrent, rightCurrent, leftYRaw, rightXRaw, true, false);
    updateStatusLight(nowMs, true, false);

    vTaskDelay(CONTROLLER_PERIOD);
  }
}

void taskSerialPrint(void *pvParameters) {
  (void)pvParameters;

  TickType_t lastWake = xTaskGetTickCount();
  for (;;) {
    DJIMotorState leftSnapshot;
    DJIMotorState rightSnapshot;
    int16_t leftCommand;
    int16_t rightCommand;
    uint32_t rxFrames;
    uint32_t unknownFrames;
    uint32_t txErrors;
    esp_err_t initStatus;
    bool canReadySnapshot;
    bool controllerConnectedSnapshot;
    bool controllerSafetyStopSnapshot;
    bool pairingModeSnapshot;
    int16_t leftStickYRawSnapshot;
    int16_t rightStickXRawSnapshot;
    float driveForwardSnapshot;
    float driveTurnSnapshot;
    float driveLeftCurrentSnapshot;
    float driveRightCurrentSnapshot;

    portENTER_CRITICAL(&motorMux);
    leftSnapshot = leftMotor;
    rightSnapshot = rightMotor;
    leftCommand = motorCommandRaw[LEFT_MOTOR_ID - 1];
    rightCommand = motorCommandRaw[RIGHT_MOTOR_ID - 1];
    rxFrames = rxFrameCount;
    unknownFrames = unknownFrameCount;
    txErrors = txErrorCount;
    initStatus = canInitStatus;
    canReadySnapshot = canReady;
    portEXIT_CRITICAL(&motorMux);

    portENTER_CRITICAL(&controlMux);
    controllerConnectedSnapshot = controllerConnected;
    controllerSafetyStopSnapshot = controllerSafetyStop;
    pairingModeSnapshot = pairingMode;
    leftStickYRawSnapshot = leftStickYRaw;
    rightStickXRawSnapshot = rightStickXRaw;
    driveForwardSnapshot = driveForward;
    driveTurnSnapshot = driveTurn;
    driveLeftCurrentSnapshot = driveLeftCurrent;
    driveRightCurrentSnapshot = driveRightCurrent;
    portEXIT_CRITICAL(&controlMux);

    // Label:value pairs are accepted by the Arduino IDE Serial Plotter.
    Serial.print("L_angle:");
    Serial.print(leftSnapshot.angleDeg, 2);
    Serial.print('\t');
    Serial.print("R_angle:");
    Serial.print(rightSnapshot.angleDeg, 2);
    Serial.print('\t');
    Serial.print("L_rpm:");
    Serial.print(leftSnapshot.rpm);
    Serial.print('\t');
    Serial.print("R_rpm:");
    Serial.print(rightSnapshot.rpm);
    Serial.print('\t');
    Serial.print("L_current:");
    Serial.print(leftSnapshot.torqueCurrent);
    Serial.print('\t');
    Serial.print("R_current:");
    Serial.print(rightSnapshot.torqueCurrent);
    Serial.print('\t');
    Serial.print("L_temp:");
    Serial.print(leftSnapshot.temperatureC);
    Serial.print('\t');
    Serial.print("R_temp:");
    Serial.print(rightSnapshot.temperatureC);
    Serial.print('\t');
    Serial.print("L_cmd:");
    Serial.print(commandToNormalized(leftCommand), 3);
    Serial.print('\t');
    Serial.print("R_cmd:");
    Serial.print(commandToNormalized(rightCommand), 3);
    Serial.print('\t');
    Serial.print("L_online:");
    Serial.print(leftSnapshot.online ? 1 : 0);
    Serial.print('\t');
    Serial.print("R_online:");
    Serial.print(rightSnapshot.online ? 1 : 0);
    Serial.print('\t');
    Serial.print("rx_frames:");
    Serial.print(rxFrames);
    Serial.print('\t');
    Serial.print("unknown_frames:");
    Serial.print(unknownFrames);
    Serial.print('\t');
    Serial.print("tx_errors:");
    Serial.print(txErrors);
    Serial.print('\t');
    Serial.print("can_status:");
    Serial.print(static_cast<int>(initStatus));
    Serial.print('\t');
    Serial.print("can_ready:");
    Serial.print(canReadySnapshot ? 1 : 0);
    Serial.print('\t');
    Serial.print("controller:");
    Serial.print(controllerConnectedSnapshot ? 1 : 0);
    Serial.print('\t');
    Serial.print("safety:");
    Serial.print(controllerSafetyStopSnapshot ? 1 : 0);
    Serial.print('\t');
    Serial.print("pairing:");
    Serial.print(pairingModeSnapshot ? 1 : 0);
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
    Serial.println(driveRightCurrentSnapshot, 3);

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
