#include "pcc.h"

const uint16_t PRECHARGE_TIME_LOWER = 700; // ms
const uint16_t PRECHARGE_TIME_UPPER = 1100; // ms
const double PRECHARGE_PERCENTAGE = 0.87;
const uint16_t TIMEOUT = 10000; // us

double getVoltage(uint8_t pin)
{
    // equation derived from data plot as F = 76.7*V + 2.93
    // therefore, V = (F - 2.93) / 76.7
    // accumulator is 100.8V max, and we'll use a 10% voltage divider, 
    // so max return value is 10.08
    double f = getFrequency(pin);
    return f / 76.7;
}

double getFrequency(uint8_t pin)
{
    const uint16_t TIMEOUT = 10000; // us
    uint16_t tHigh = pulseIn(pin, HIGH, TIMEOUT);
    uint16_t tLow = pulseIn(pin, LOW, TIMEOUT);
    if (tHigh == 0 || tLow == 0){
      return 0;
    }
    return 1000000.0 / (double)(tHigh + tLow);    // f = 1/T
}

uint8_t prechargeSequence(uint8_t tsVoltagePin, uint8_t accVoltagePin, uint8_t prechargeRelayPin, uint8_t bPosRelayPin)
{
    delay(500);
    digitalWrite(bPosRelayPin, LOW);
    digitalWrite(prechargeRelayPin, HIGH);
    uint32_t start = millis();
    delay(50);
    double targetV = getVoltage(accVoltagePin) * 10 * PRECHARGE_PERCENTAGE;
    double currV = getVoltage(tsVoltagePin) * 10;
    Serial.print("targetV: ");
    Serial.print(targetV);
    Serial.print(" voltage ts: ");
    Serial.println(currV);
    // delay(1);
    while(currV <= targetV){
        Serial.print(currV);
        Serial.print(" out of ");
        Serial.print(targetV);
        Serial.print(" (");
        Serial.print(currV/targetV*100);
        Serial.println("%)");
        currV = getVoltage(tsVoltagePin) * 10;
        delay(50);
    }
    uint32_t duration = millis() - start;
    Serial.println(duration);
    if(duration <= PRECHARGE_TIME_UPPER && duration >= PRECHARGE_TIME_LOWER) return 1;
    else{
        Serial.println("Error!");
        return 0;
    }
}

uint8_t prechargeSequenceTest(uint8_t tsVoltagePin, uint8_t accVoltagePin, uint8_t prechargeRelayPin)
{
    delay(5000);
    digitalWrite(prechargeRelayPin, HIGH);
    delay(50);
    double targetV = getVoltage(accVoltagePin) * 10 * PRECHARGE_PERCENTAGE;
    double currV = getVoltage(tsVoltagePin) * 10;
    uint32_t start = millis();
    delay(1);
    while(currV <= targetV){
        currV = getVoltage(tsVoltagePin) * 10;
        Serial.print(currV);
        Serial.print(" out of ");
        Serial.print(targetV);
        Serial.print(" (");
        Serial.print(currV/targetV*100);
        Serial.println("%)");
        delay(50);
    }
    uint32_t duration = millis() - start;
    Serial.println(duration);
    digitalWrite(prechargeRelayPin, LOW);
    return 1;
}