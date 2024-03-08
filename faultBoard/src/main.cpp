#include <Arduino.h>
#include <ChRt.h>
#include <FlexCAN_T4.h>
#include "pcc.h"

enum State
{
  STANDBY,
  PRECHARGING,
  TRACTIVE_SYSTEM_ACTIVE,
  READY_TO_DRIVE,
  DRIVING,
  FAULT
};

struct sensorValues {
  uint16_t throttle1Value;
  uint16_t throttle2Value;
  uint16_t brakeValue;
  uint8_t tsValue;
  bool keyValue;
};

sensorValues data;

String stateNames[] = {"STANDBY", "TRACTIVE_SYSTEM_ACTIVE", "READY_TO_DRIVE", "DRIVING", "FAULT"};

MUTEX_DECL(stateMutex);
MUTEX_DECL(appsMutex);
MUTEX_DECL(bseMutex);
MUTEX_DECL(appsPlausMutex);

// struct can_frame canMsg;

const uint16_t POT_MAX = 1023;

uint8_t MCPin = 3;
uint8_t throttlePin = A9;
uint8_t buzzerPin = 4;

uint8_t tsVoltagePin = 22;
uint8_t accVoltagePin = 23;

uint8_t prechargeRelayPin = 12;
uint8_t bPosRelayPin = 13;

uint8_t prechargeSwitch = 0;

uint16_t throttle1 = 0;
uint16_t throttle2 = 0;
uint16_t brake = 0;
uint8_t tractiveSystemActiveValue = 0;
uint8_t keySwitchValue = 0;

float brake_low = 0.5;
float brake_high = 4.5;

uint8_t MC = 0;

uint32_t faultTime = 0;
uint8_t throttleOut = 0;

State CURR_STATE = STANDBY;
// change

uint8_t appsFault = 0;
uint8_t bseFault = 0;
uint8_t appsPlausFault = 0;

FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> faultCAN;

static THD_WORKING_AREA(waThread1, 64);

static THD_FUNCTION(throttleCheck, arg)
{
  (void)arg;

  while (true)
  {
    if (abs(throttle1 - throttle2) <= POT_MAX / 10)
    {
      faultTime = millis();
    }
    if (millis() - faultTime >= 100)
    {
      chMtxLock(&appsMutex);
      appsFault = true;
      chMtxUnlock(&appsMutex);
    }
    else
    {
      chMtxLock(&appsMutex);
      appsFault = false;
      chMtxUnlock(&appsMutex);
    }
  }
}

float map_value(uint16_t aMax, uint16_t bMax, float inValue)
{
  // maps value in range a to range b
  return ((float)bMax / aMax) * (inValue);
}

static THD_WORKING_AREA(waThread2, 64);

static THD_FUNCTION(brakeCheck, arg)
{
  (void)arg;
  while (true)
  {
    float brakeVal = map_value(POT_MAX, 5, brake);
    chMtxLock(&bseMutex);
    bseFault = brakeVal < 0.5 || brakeVal > 4.5;
    chMtxUnlock(&bseMutex);
  }
}

static THD_WORKING_AREA(waThread3, 64);

static THD_FUNCTION(rtd, arg)
{
  (void)arg;
  while (true)
  {
    chMtxLock(&stateMutex);

    if (CURR_STATE == STANDBY)
    {
      if (tractiveSystemActiveValue == 1)
      {
        CURR_STATE = TRACTIVE_SYSTEM_ACTIVE;
      }
    }
    else if (CURR_STATE == TRACTIVE_SYSTEM_ACTIVE)
    {
      if (map_value(POT_MAX, 5, brake) > brake_low && map_value(POT_MAX, 5, brake) < brake_high)
      {
        CURR_STATE = READY_TO_DRIVE;
        tone(buzzerPin, 1000, 3000);
      }
      else if (tractiveSystemActiveValue == 0)
      {
        CURR_STATE = STANDBY;
        tone(buzzerPin, 0);
      }
    }
    else if (CURR_STATE == READY_TO_DRIVE)
    {
      if (keySwitchValue == 1)
      {
        CURR_STATE = DRIVING;
      }
      else if (tractiveSystemActiveValue == 0)
      {
        CURR_STATE = STANDBY;
        tone(buzzerPin, 0);
      }
      else if (map_value(POT_MAX, 5, brake) <= brake_low)
      {
        CURR_STATE = TRACTIVE_SYSTEM_ACTIVE;
      }
    }
    else if (CURR_STATE == DRIVING)
    {
      if (tractiveSystemActiveValue == 0)
      {
        CURR_STATE = STANDBY;
      }
      else if (keySwitchValue == 0)
      {
        CURR_STATE = READY_TO_DRIVE;
      }
    }

    chMtxUnlock(&stateMutex);
  }
}

static THD_WORKING_AREA(waThread4, 64);

static THD_FUNCTION(read, arg)
{
  (void)arg;

  // CAN_message_t msg;
  // while (true){
  //   if (faultCAN.read(msg) && msg.id == 0x036){
  //     // Serial.println("receiving!");
  //     memcpy(&data, msg.buf, sizeof(data));
  //     Serial.print(data.throttle1Value * 100 / POT_MAX);
  //     Serial.print("% throttle2: ");
  //     Serial.print(data.throttle2Value * 100 / POT_MAX);
  //     Serial.print("% brake value: ");
  //     Serial.print(map_value(POT_MAX, 5, data.brakeValue));
  //     Serial.print(" current state: ");
  //     Serial.println(stateNames[CURR_STATE]);
  //   }
  // }

    // if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK)
    // {
    //   throttle1 = (uint16_t)canMsg.data[0] << 8 | canMsg.data[1];
    //   throttle2 = (uint16_t)canMsg.data[2] << 8 | canMsg.data[3];
    //   brake = (uint16_t)canMsg.data[4] << 8 | canMsg.data[5];
    //   tractiveSystemActiveValue = canMsg.data[6];
    //   keySwitchValue = canMsg.data[7];
    //   Serial.print("throttle1: ");
    //   Serial.print(throttle1 * 100 / POT_MAX);
    //   Serial.print("% throttle2: ");
    //   Serial.print(throttle2 * 100 / POT_MAX);
    //   Serial.print("% brake value: ");
    //   Serial.print(map_value(POT_MAX, 5, brake));
    //   Serial.print(" current state: ");
    //   Serial.println(stateNames[CURR_STATE]);
    // }
}

static THD_WORKING_AREA(waThread5, 64);

static THD_FUNCTION(write, arg)
{
  (void)arg;
  while (true)
  {
    chMtxLock(&bseMutex);
    uint8_t currBse = bseFault;
    chMtxUnlock(&bseMutex);

    chMtxLock(&appsMutex);
    uint8_t currApps = appsFault;
    chMtxUnlock(&appsMutex);

    chMtxLock(&appsPlausMutex);
    uint8_t currAppsPlaus = appsPlausFault;
    chMtxUnlock(&appsPlausMutex);

    if (currBse || currApps || currAppsPlaus)
    {
      chMtxLock(&stateMutex);
      if (CURR_STATE == DRIVING)
      {
        CURR_STATE = FAULT;
      }
      chMtxUnlock(&stateMutex);
      MC = 0;
      throttleOut = 0;
    }
    else
    {
      chMtxLock(&stateMutex);
      if (CURR_STATE == FAULT)
      {
        CURR_STATE = DRIVING;
      }
      chMtxUnlock(&stateMutex);
      MC = 1;
      throttleOut = ((throttle1 + throttle2) / 2) / 4;
    }

    digitalWrite(MCPin, MC);
    analogWrite(throttlePin, throttleOut);
  }
}

static THD_WORKING_AREA(waThread6, 64);

static THD_FUNCTION(appsPlaus, arg)
{
  (void)arg;
  while (true)
  {
    float currThrottle = map_value(POT_MAX, 100, (throttle1 + throttle2) / 2);
    float currBrake = map_value(POT_MAX, 5, brake);
    chMtxLock(&appsPlausMutex);
    if (appsPlausFault)
      appsPlausFault = !(currThrottle < 5); // true is good, 0 is fault off
    else
      appsPlausFault = currThrottle > 25 && currBrake > 0.5;
    chMtxUnlock(&appsPlausMutex);
  }
}

static THD_WORKING_AREA(waThread7, 64);

static THD_FUNCTION(pcc, arg){
  (void)arg;
  // while(true){
  //   digitalWrite(prechargeRelayPin, HIGH);
  //   delay(100);
  //   digitalWrite(bPosRelayPin, HIGH);
  //   delay(100);
  //   digitalWrite(prechargeRelayPin, LOW);
  //   delay(100);
  //   digitalWrite(prechargeRelayPin, HIGH);
  //   delay(100);
  //   digitalWrite(bPosRelayPin, LOW);
  //   delay(100);
  //   digitalWrite(prechargeRelayPin, LOW);
  //   delay(100);
  //   digitalWrite(bPosRelayPin, HIGH);
  //   delay(100);
  //   digitalWrite(bPosRelayPin, LOW);
  //   delay(100);
  // }
  while(true){
    int switchVal = digitalRead(prechargeSwitch);
    if(CURR_STATE == FAULT && switchVal == LOW){
      CURR_STATE = STANDBY;
      delay(1000); //debounce kinda
    }
    else if(CURR_STATE == STANDBY && switchVal == HIGH){
      CURR_STATE = PRECHARGING;
      int status = prechargeSequence(tsVoltagePin, accVoltagePin, prechargeRelayPin, bPosRelayPin);
      if(status){
        CURR_STATE = TRACTIVE_SYSTEM_ACTIVE;
        digitalWrite(bPosRelayPin, HIGH);
        delay(200);
        digitalWrite(prechargeRelayPin, LOW);
      }
      else{
        CURR_STATE = FAULT;
        digitalWrite(prechargeRelayPin, LOW);
        digitalWrite(bPosRelayPin, LOW);
      }
      delay(1000);
    }
    else if(CURR_STATE == TRACTIVE_SYSTEM_ACTIVE && switchVal == LOW){
      CURR_STATE = STANDBY;
      digitalWrite(prechargeRelayPin, LOW);
      digitalWrite(bPosRelayPin, LOW);
      delay(1000);
    }
  }
  // int state = prechargeSequence(tsVoltagePin, accVoltagePin, prechargeRelayPin, bPosRelayPin);
  // while(true){
  //   if(state){
  //       digitalWrite(bPosRelayPin, HIGH);
  //       delay(200);
  //       digitalWrite(prechargeRelayPin, LOW);
  //   }
  //   else{
  //       digitalWrite(prechargeRelayPin, LOW);
  //       digitalWrite(bPosRelayPin, LOW);
  //   }
  // }
}


void chSetup()
{
  // Start threads.
  // chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, throttleCheck, NULL);
  // chThdCreateStatic(waThread2, sizeof(waThread2), NORMALPRIO, brakeCheck, NULL);
  // chThdCreateStatic(waThread3, sizeof(waThread3), NORMALPRIO, rtd, NULL);
  chThdCreateStatic(waThread4, sizeof(waThread4), NORMALPRIO, read, NULL);
  // chThdCreateStatic(waThread5, sizeof(waThread5), NORMALPRIO, write, NULL);
  // chThdCreateStatic(waThread6, sizeof(waThread6), NORMALPRIO, appsPlaus, NULL);
  chThdCreateStatic(waThread7, sizeof(waThread7), NORMALPRIO+1, pcc, NULL);
}

void setup()
{
  Serial.begin(9600);
  pinMode(MCPin, OUTPUT);
  digitalWrite(MCPin, MC);
  pinMode(throttlePin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(tsVoltagePin, INPUT);
  pinMode(accVoltagePin, INPUT);
  pinMode(prechargeRelayPin, OUTPUT);
  pinMode(bPosRelayPin, OUTPUT);
  pinMode(prechargeSwitch, INPUT);

  if(digitalRead(prechargeSwitch) == HIGH){
    CURR_STATE = FAULT; // to ensure that the precharge sequence is not run right as power is provided
    digitalWrite(prechargeRelayPin, LOW);
    digitalWrite(bPosRelayPin, LOW);
  }

  // CANFD_timings_t config;
  // config.clock = CLK_24MHz;
  // config.baudrate = 1000000;
  // config.baudrateFD = 2000000;
  // config.propdelay = 190;
  // config.bus_length = 1;
  // config.sample = 70;
  faultCAN.begin();
  faultCAN.setBaudRate(250000);
  chBegin(&chSetup);

}

// FAULT BOARD
void loop()
{
}