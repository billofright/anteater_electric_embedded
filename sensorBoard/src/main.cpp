#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>
#include <Arduino_FreeRTOS.h>


struct can_frame canMsg;

uint8_t pot1Pin = A7;
uint8_t pot2Pin = A6;

MCP2515 mcp2515;

void pot1(void *pvParameters);
void pot2(void *pvParameters);
void send(void *pvParameters);

uint16_t pot1Value = 0;
uint16_t pot2Value = 0;


void setup()
{
  Serial.begin(9600);
  SPI.begin();

  mcp2515.init(10); // CS pin as 10

  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS);
  mcp2515.setNormalMode();

  xTaskCreate(pot1, "Pot 1", 100, nullptr, 1, nullptr);
  xTaskCreate(pot2, "Pot 2", 100, nullptr, 1, nullptr);
  xTaskCreate(send, "Send", 100, nullptr, 1, nullptr);
  vTaskStartScheduler();

  pinMode(pot1Pin, INPUT);
  pinMode(pot2Pin, INPUT);
}

void loop() {}

void pot1(void *pvParameters) {
  while (true) {
    pot1Value = analogRead(pot1Pin);
  }
}

void pot2(void *pvParameters) {
  while (true) {
    pot2Value = analogRead(pot2Pin);
  }
}

void send(void *pvParameters) {
  while (true) {
    canMsg.can_id = 0x036; // CAN id as 0x036
    canMsg.can_dlc = 8;    // CAN data length as 8
    canMsg.data[0] = pot1Value >> 8; 
    canMsg.data[1] = pot1Value & 0xFF; 
    canMsg.data[2] = pot2Value >> 8;; 
    canMsg.data[3] = pot2Value & 0xFF;
    canMsg.data[4] = 0x05;
    canMsg.data[5] = 0x06;
    canMsg.data[6] = 0x07;
    canMsg.data[7] = 0x08;
  
    mcp2515.sendMessage(&canMsg); // Sends the CAN message
    delay(10);
  }
}