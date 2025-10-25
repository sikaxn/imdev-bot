#include <Arduino.h>
#include "wit_c_sdk.h"
#include "REG.h"

// ESP32 UART2 pins for the sensor
static const int UART_TX_PIN = 23;   // IO23
static const int UART_RX_PIN = 19;   // IO19
static const uint32_t WT_BAUD = 115200;

// HAL shims for Wit SDK
static void SerialWriteFunc(uint8_t* data, uint32_t len) { Serial2.write(data, len); }
static void DelayMs(uint16_t ms) { delay(ms); }

// Called by SDK when new registers are updated
static void SensorDataUpdate(uint32_t uiReg, uint32_t uiRegNum) {
  // Print when we get AX block, GX block, or ANGLE block
  if (uiReg == AX || uiReg == GX || uiReg == Roll) {
    const float ax = (float)sReg[AX]    / 32768.0f * 16.0f;
    const float ay = (float)sReg[AY]    / 32768.0f * 16.0f;
    const float az = (float)sReg[AZ]    / 32768.0f * 16.0f;

    const float gx = (float)sReg[GX]    / 32768.0f * 2000.0f;
    const float gy = (float)sReg[GY]    / 32768.0f * 2000.0f;
    const float gz = (float)sReg[GZ]    / 32768.0f * 2000.0f;

    const float roll  = (float)sReg[Roll]  / 32768.0f * 180.0f;
    const float pitch = (float)sReg[Pitch] / 32768.0f * 180.0f;
    const float yaw   = (float)sReg[Yaw]   / 32768.0f * 180.0f;

    Serial.print("ACC[g] ");
    Serial.printf("%6.2f %6.2f %6.2f | ", ax, ay, az);
    Serial.print("GYRO[dps] ");
    Serial.printf("%7.2f %7.2f %7.2f | ", gx, gy, gz);
    Serial.print("ANGLE[deg] ");
    Serial.printf("%7.2f %7.2f %7.2f\n", roll, pitch, yaw);
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Srial.println("\nWT901 (JY901) UART reader on ESP32: TX=IO23, RX=IO19 @ 115200");
e
  // Start UART2 for sensor
  Serial2.begin(WT_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

  // Init Wit SDK in "normal" UART protocol
  WitInit(WIT_PROTOCOL_NORMAL, 0x50);
  WitSerialWriteRegister(SerialWriteFunc);
  WitDelayMsRegister(DelayMs);
  WitRegisterCallBack(SensorDataUpdate);

  // Optional: Request content if your module needs it (many WT901 stream by default)
  // WitSetContent(RSW_ACC | RSW_GYRO | RSW_ANGLE);    // enable blocks
  // WitSetOutputRate(RRATE_50HZ);                     // set output rate

  Serial.println("Wit SDK ready. Waiting for data...");
}

void loop() {
  // Feed bytes from sensor into the SDK parser
  while (Serial2.available()) {
    uint8_t b = (uint8_t)Serial2.read();
    WitSerialDataIn(b);
  }
  delay(1);
}
