#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>
#include <Arduino_FreeRTOS.h>

struct can_frame canMsg;

uint8_t throttle1Pin = 25;
uint8_t throttle2Pin = 26;
uint8_t brake1Pin;
uint8_t brake2Pin;
uint8_t wheel1Pin;
uint8_t wheel2Pin;
// uint8_t tsSwitchPin = 5;
// uint8_t pushButtonPin = 6;

MCP2515 mcp2515;

struct sensorValues {
  uint16_t throttle1Value;
  uint16_t throttle2Value;
  uint16_t brake1Value;
  uint16_t brake2Value;
  uint16_t wheel1Value;
  uint16_t wheel2Value;
  uint8_t tsValue;
  bool keyValue;
};

void throttle1(void *pvParameters);
void throttle2(void *pvParameters);
void brake1(void *pvParameters);
void brake2(void *pvParameters);
void wheel1(void *pvParameters);
void wheel2(void *pvParameters);
void send(void *pvParameters);
void ts(void *pvParameters);
void key(void *pvParameters);

sensorValues sensorVals{0, 0, 0, 0, 0};


void setup()
{
  Serial.begin(9600);
  SPI.begin();

  mcp2515.init(10); // CS pin as 10

  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS);
  mcp2515.setNormalMode();

  xTaskCreate(throttle1, "Throttle1", 100, nullptr, 1, nullptr);
  xTaskCreate(throttle2, "Throttle2", 100, nullptr, 1, nullptr);
  xTaskCreate(brake1, "Brake1", 100, nullptr, 1, nullptr);
  xTaskCreate(brake2, "Brake2", 100, nullptr, 1, nullptr);
  xTaskCreate(wheel1, "Wheel 1", 100, nullptr, 1, nullptr);
  xTaskCreate(wheel2, "Wheel 2", 100, nullptr, 1, nullptr);
  xTaskCreate(send, "Send", 100, nullptr, 1, nullptr);
  xTaskCreate(ts, "TS Switch", 100, nullptr, 1, nullptr);
  xTaskCreate(key, "Key Switch", 100, nullptr, 1, nullptr);
  vTaskStartScheduler();

  pinMode(throttle1Value, INPUT);
  pinMode(throttle2Value, INPUT);
  pinMode(tsSwitchPin, INPUT);
  pinMode(pushButtonPin, INPUT);
  pinMode(brake1Value, INPUT);
  pinMode(brake2Value, INPUT);
  pinMode(wheel1Value, INPUT);
  pinMode(wheel2Value, INPUT);
  pinMode(tsValue, INPUT);
  pinMode(keyValue, INPUT);
}

void loop() {}

void throttle1(void *pvParameters)
{
  while (true)
  {
    sensorVals.throttle1Value = analogRead(throttle1Pin);
  }
}

void wheel1(void *pvParameters)
{
  while (true)
  {
    sensorVals.wheel1Value = analogRead(wheel1Pin);
  }
}

void wheel2(void *pvParameters)
{
  while (true)
  {
    sensorVals.wheel2Value = analogRead(wheel2Pin);
  }
}


void throttle2(void *pvParameters)
{
  while (true)
  {
    sensorVals.throttle2Value = analogRead(throttle2Pin);
  }
}

void brake1(void *pvParameteres)
{
  while (true)
  {
    sensorVals.brakeValue = analogRead(brake1Pin);
  }
}

void brake2(void *pvParameteres)
{
  while (true)
  {
    sensorVals.brakeValue = analogRead(brake2Pin);
  }
}

void ts(void *pvParameteres)
{
  while (true)
  {
    sensorVals.tsValue = digitalRead(tsSwitchPin);
  }
}

void key(void *pvParameteres)
{
  uint32_t lastPressed = millis();
  uint8_t prevValue = LOW;
  uint16_t debounceDelay = 500;

  while (true)
  {
    int currValue = digitalRead(pushButtonPin);
    if(currValue == HIGH && prevValue == LOW && millis() - lastPressed > debounceDelay){
      lastPressed = millis();
      sensorVals.keyValue = !sensorVals.keyValue;
    }
    prevValue = currValue;
  }
}

void send(void *pvParameters)
{
  while (true)
  {
    canMsg.can_id = 0x036; // CAN id as 0x036
    canMsg.can_dlc = 14;    // CAN data length as 8
    canMsg.data[0] = sensorVals.throttle1Value >> 8;
    canMsg.data[1] = sensorVals.throttle1Value & 0xFF;
    canMsg.data[2] = sensorVals.throttle2Value >> 8;
    canMsg.data[3] = sensorVals.throttle2Value & 0xFF;
    canMsg.data[4] = sensorVals.brake1Value >> 8;
    canMsg.data[5] = sensorVals.brake1Value & 0xFF;
    canMsg.data[6] = sensorVals.brake2Value >> 8;
    canMsg.data[7] = sensorVals.brake2Value & 0xFF;
    canMsg.data[8] = sensorVals.tsValue;
    canMsg.data[9] = sensorVals.keyValue;
    canMsg.data[10] = sensorVals.wheel1Value >> 8;
    canMsg.data[11] = sensorVals.wheel1Value & 0xFF;
    canMsg.data[12] = sensorVals.wheel2Value >> 8;
    canMsg.data[13] = sensorVals.wheel2Value & 0xFF;


    mcp2515.sendMessage(&canMsg); // Sends the CAN message
    delay(10);

    Serial.print("tsValue: " );
    Serial.println(sensorVals.tsValue);
    Serial.print("keyValue: " );
    Serial.println(sensorVals.keyValue);
  }
}