#include <pcc.h>

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
    const u_int16_t TIMEOUT = 0;
    uint16_t tHigh = pulseIn(pin, HIGH, TIMEOUT);
    uint16_t tLow = pulseIn(pin, LOW, TIMEOUT);
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