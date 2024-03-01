#include "pcc.h"

double getVoltage(uint8_t pin)
{
    // equation derived from data plot as F = 76.7*V + 2.93
    // therefore, V = (F - 2.93) / 76.7
    // accumulator is 100.8V max, and we'll use a 10% voltage divider, 
    // so max return value is 10.08
    double f = getFrequency(pin);
    return (f - 2.93) / 76.7;
}

double getFrequency(uint8_t pin)
{
    uint16_t tHigh = pulseIn(pin, HIGH);
    uint16_t tLow = pulseIn(pin, LOW);
    if (tHigh == 0 || tLow == 0){
      return 0;
    }
    return 1000000.0 / (double)(tHigh + tLow);    // f = 1/T
}

uint8_t prechargeSequence(uint8_t tsVoltagePin, uint8_t accVoltagePin, uint8_t prechargeRelayPin, uint8_t bPosRelayPin)
{
    digitalWrite(prechargeRelayPin, HIGH);
    delay(100);
    uint32_t start = millis();
    while(getVoltage(tsVoltagePin) < 10.08){
        delay(100);
    }
    uint32_t duration = millis() - start;
    if(duration > PRECHARGE_TIME - PRECHARGE_TIME_RANGE && duration < PRECHARGE_TIME + PRECHARGE_TIME_RANGE){
        digitalWrite(bPosRelayPin, HIGH);
        delay(100);
        digitalWrite(prechargeRelayPin, LOW);
        return 1;
    }
    else{
        digitalWrite(prechargeRelayPin, LOW);
        return 0;
    }
}

uint8_t prechargeSequenceTest(uint8_t tsVoltagePin, uint8_t prechargeRelayPin, uint8_t bPosRelayPin)
{
    delay(3000);
    digitalWrite(prechargeRelayPin, HIGH);
    delay(1);
    uint32_t start = millis();
    double currV = getVoltage(tsVoltagePin);
    int currTime = 1;
    while(currV < 5.3){
        if(millis() - start > 2000) break;
        Serial.print("voltage at ");
        Serial.print(currTime += 100);
        Serial.print("ms: ");
        Serial.println(currV);
        currV = getVoltage(tsVoltagePin);
        delay(100);
    }
    uint32_t duration = millis() - start;
    Serial.println(duration/1000.0);
    if(duration <= 2000 && duration >= 1000){
        digitalWrite(bPosRelayPin, HIGH);
        delay(100);
        digitalWrite(prechargeRelayPin, LOW);
        return 1;
    }
    else{
        Serial.println("Error!");
        digitalWrite(prechargeRelayPin, LOW);
        return 0;
    }
}