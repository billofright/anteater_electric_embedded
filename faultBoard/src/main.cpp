#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>
#include <ChRt.h>

struct can_frame canMsg;

const uint16_t POT_MAX = 1023;

uint8_t MCPin = 3;
uint8_t throttlePin = A9;

uint16_t throttle1 = 0;
uint16_t throttle2 = 0;
uint16_t brake = 0;

uint8_t MC = 0;

MCP2515 mcp2515;

uint32_t faultTime = 0;
uint8_t throttleOut = 0;

static THD_WORKING_AREA(waThread1, 64);

static THD_FUNCTION(Thread1, arg) {
  (void)arg;
  while(true) {
    if (millis() - faultTime >= 100)
    {
      MC = 0;
      throttleOut = 0;
    }
    else
    {
      MC = 1;
      throttleOut = ((throttle1 + throttle2) / 2) / 4;
    }

    if (abs(throttle1 - throttle2) <= POT_MAX / 10)
    {
      faultTime = millis();
    }

    if (brake > map_value(5, POT_MAX, 0.5) && brake < map_value(5, POT_MAX, 4.5))
      MC = 1;
    else
      MC = 0;

    if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK)
    {
      throttle1 = (uint16_t)canMsg.data[0] << 8 | canMsg.data[1];
      throttle2 = (uint16_t)canMsg.data[2] << 8 | canMsg.data[3];
      brake = (uint16_t)canMsg.data[4] << 8 | canMsg.data[5];
      Serial.println("throttle1: " + String(throttle1) + " throttle2: " + String(throttle2) + " Brake: " + String(brake) + " MC: " + String(MC) + " Throttle: " + String(throttleOut) + " brake value: " + String(map_value(5, POT_MAX, 4.5)));
    }
  }
}

static THD_WORKING_AREA(waThread2, 64);

static THD_FUNCTION(Thread2, arg) {
  (void)arg;
  while (true) {
    digitalWrite(MCPin, MC);
    analogWrite(throttlePin, throttleOut);
  }
}

// static THD_WORKING_AREA(waThread3, 64);

// static THD_FUNCTION(Thread3, arg) {
//   (void)arg;
//    while (true) {
//     chThdSleepMilliseconds(100);
//     Serial.println("Hello2");
//   }
// }

float map_value(uint16_t aMax, uint16_t bMax, float inValue)
{
  // maps value in range a to range b
  return ((float)bMax / aMax) * (inValue);
}

void chSetup() {
  // Start threads.
  chThdCreateStatic(waThread1, sizeof(waThread1),
    NORMALPRIO, Thread1, NULL);

  chThdCreateStatic(waThread2, sizeof(waThread2),
    NORMALPRIO, Thread2, NULL);
  
  // chThdCreateStatic(waThread3, sizeof(waThread3),
  //   NORMALPRIO, Thread3, NULL);

}

void setup()
{
  Serial.begin(9600);
  SPI.begin(); // Begins SPI communication

  mcp2515.init(10);

  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS);
  mcp2515.setNormalMode();

  pinMode(MCPin, OUTPUT);
  digitalWrite(MCPin, MC);
  pinMode(throttlePin, OUTPUT);

  chBegin(chSetup);
}

// FAULT BOARD
void loop()
{
  // if (millis() - faultTime >= 100)
  // {
  //   MC = 0;
  //   throttleOut = 0;
  // }
  // else
  // {
  //   MC = 1;
  //   throttleOut = ((throttle1 + throttle2) / 2) / 4;
  // }

  // if (abs(throttle1 - throttle2) <= POT_MAX / 10)
  // {
  //   faultTime = millis();
  // }

  // if (brake > map_value(5, POT_MAX, 0.5) && brake < map_value(5, POT_MAX, 4.5))
  //   MC = 1;
  // else
  //   MC = 0;

  // if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK)
  // {
  //   throttle1 = (uint16_t)canMsg.data[0] << 8 | canMsg.data[1];
  //   throttle2 = (uint16_t)canMsg.data[2] << 8 | canMsg.data[3];
  //   brake = (uint16_t)canMsg.data[4] << 8 | canMsg.data[5];
  //   Serial.println("throttle1: " + String(throttle1) + " throttle2: " + String(throttle2) + " Brake: " + String(brake) + " MC: " + String(MC) + " Throttle: " + String(throttleOut) + " brake value: " + String(map_value(5, POT_MAX, 4.5)));
  // }

  // digitalWrite(MCPin, MC);
  // analogWrite(throttlePin, throttleOut);
}