#ifndef PCC_H
#define PCC_H

#include <Arduino.h>

const uint16_t PRECHARGE_TIME = 5000;
const uint16_t PRECHARGE_TIME_RANGE = 500;

double getVoltage(uint8_t pin);
double getFrequency(uint8_t pin);
uint8_t prechargeSequence(uint8_t tsVoltagePin, uint8_t accVoltagePin, uint8_t prechargeRelayPin, uint8_t bPosRelayPin);

#endif