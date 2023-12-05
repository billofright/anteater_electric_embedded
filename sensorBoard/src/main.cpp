#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>
#include <Arduino_FreeRTOS.h>

struct can_frame canMsg;

uint8_t throttle1Pin = A7;
uint8_t throttle2Pin = A6;
uint8_t brakePin = A5;
uint8_t tsSwitchPin = 5;
uint8_t keySwitchPin = 6;

MCP2515 mcp2515;

void throttle1(void *pvParameters);
void throttle2(void *pvParameters);
void brake(void *pvParameters);
void send(void *pvParameters);
void ts(void *pvParameters);
void key(void *pvParameters);

uint16_t throttle1Value = 0;
uint16_t throttle2Value = 0;
uint16_t brakeValue = 0;
uint8_t tsValue = 0;
uint8_t keyValue = 0;

void setup()
{
  Serial.begin(9600);
  SPI.begin();

  mcp2515.init(10); // CS pin as 10

  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS);
  mcp2515.setNormalMode();

  xTaskCreate(throttle1, "Pot 1", 100, nullptr, 1, nullptr);
  xTaskCreate(throttle2, "Pot 2", 100, nullptr, 1, nullptr);
  xTaskCreate(brake, "Brake", 100, nullptr, 1, nullptr);
  xTaskCreate(send, "Send", 100, nullptr, 1, nullptr);
  xTaskCreate(ts, "TS Switch", 100, nullptr, 1, nullptr);
  xTaskCreate(key, "Key Switch", 100, nullptr, 1, nullptr);
  vTaskStartScheduler();

  pinMode(throttle1Pin, INPUT);
  pinMode(throttle2Pin, INPUT);
  pinMode(tsSwitchPin, INPUT);
  pinMode(keySwitchPin, INPUT);

}

void loop() {}

void throttle1(void *pvParameters)
{
  while (true)
  {
    throttle1Value = analogRead(throttle1Pin);
  }
}

void throttle2(void *pvParameters)
{
  while (true)
  {
    throttle2Value = analogRead(throttle2Pin);
  }
}

void brake(void *pvParameteres)
{
  while (true)
  {
    brakeValue = analogRead(brakePin);
  }
}

void ts(void *pvParameteres)
{
  while (true)
  {
    tsValue = digitalRead(tsSwitchPin);
  }
}

void key(void *pvParameteres)
{
  while (true)
  {
    keyValue = digitalRead(keySwitchPin);
  }
}

void send(void *pvParameters)
{
  while (true)
  {
    canMsg.can_id = 0x036; // CAN id as 0x036
    canMsg.can_dlc = 8;    // CAN data length as 8
    canMsg.data[0] = throttle1Value >> 8;
    canMsg.data[1] = throttle1Value & 0xFF;
    canMsg.data[2] = throttle2Value >> 8;
    canMsg.data[3] = throttle2Value & 0xFF;
    canMsg.data[4] = brakeValue >> 8;
    canMsg.data[5] = brakeValue & 0xFF;
    canMsg.data[6] = tsValue;
    canMsg.data[7] = keyValue;

    mcp2515.sendMessage(&canMsg); // Sends the CAN message
    delay(10);

    Serial.print("tsValue: " );
    Serial.println(tsValue);
    Serial.print("keyValue: " );
    Serial.println(keyValue);
  }
}