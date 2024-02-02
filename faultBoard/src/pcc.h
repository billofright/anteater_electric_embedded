#ifndef PCC_H
#define PCC_H

#include <Arduino.h>

const uint16_t PRECHARGE_TIME = 5000;
const uint16_t PRECHARGE_TIME_RANGE = 500;
const u_int16_t TIMEOUT = 0;

const double FREQUENCY = 2.93;
const double VOLTAGE = 76.7;
const double MAXVOLTAGE = 10.08;

double getVoltage(uint8_t pin);
double getFrequency(uint8_t pin);
uint8_t prechargeSequence(uint8_t tsVoltagePin, uint8_t accVoltagePin, uint8_t prechargeRelayPin, uint8_t bPosRelayPin);

#endif