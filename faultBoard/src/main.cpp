#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>
#include <chrono>

struct can_frame canMsg;

const uint16_t POT_MAX = 1023;

uint8_t ledPin = 3;

uint16_t pot1 = 0;
uint16_t pot2 = 0;

uint8_t led = 0;

MCP2515 mcp2515;

uint32_t faultTime;


void setup(){
  Serial.begin(9600);
  SPI.begin(); //Begins SPI communication

  mcp2515.init(10);

  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS);
  mcp2515.setNormalMode();

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, led);

}

// FAULT BOARD
void loop() {
  if (millis() - faultTime >= 3000){
    led = 0;
  }
  else {
    led = 1;
  }

  if ((((float) pot1)/POT_MAX < 0.05 || ((float) pot2)/POT_MAX < 0.05) || ((float) abs(pot1 - pot2)) / max(pot1, pot2) <= 0.1){
    faultTime = millis();
  }

  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    pot1 = (uint16_t)canMsg.data[0] << 8 | canMsg.data[1];
    pot2 = (uint16_t)canMsg.data[2] << 8 | canMsg.data[3];
    
    Serial.println("Pot1: " + String(pot1) + " Pot2: " + String(pot2) + " LED: " + String(led));
  } 

  digitalWrite(ledPin, led);  

}