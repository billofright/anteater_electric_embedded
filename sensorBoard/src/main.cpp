#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>

struct can_frame canMsg;

uint8_t pot1Pin = A7;
uint8_t pot2Pin = A6;

MCP2515 mcp2515;

void setup()
{
  Serial.begin(9600);
  SPI.begin();

  mcp2515.init(10); // CS pin as 10

  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS);
  mcp2515.setNormalMode();

  pinMode(pot1Pin, INPUT);
  pinMode(pot2Pin, INPUT);
}

// SENSOR BOARD
void loop()
{

  uint16_t pot1 = analogRead(pot1Pin);
  uint16_t pot2 = analogRead(pot2Pin);

  canMsg.can_id = 0x036; // CAN id as 0x036
  canMsg.can_dlc = 8;    // CAN data length as 8
  canMsg.data[0] = pot1 >> 8; 
  canMsg.data[1] = pot1 & 0xFF; 
  canMsg.data[2] = pot2 >> 8;; 
  canMsg.data[3] = pot2 & 0xFF;
  canMsg.data[4] = 0x05;
  canMsg.data[5] = 0x06;
  canMsg.data[6] = 0x07;
  canMsg.data[7] = 0x08;
  
  mcp2515.sendMessage(&canMsg); // Sends the CAN message
  delay(100);
}