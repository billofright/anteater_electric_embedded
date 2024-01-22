#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>
#include <Arduino_FreeRTOS.h>

struct can_frame canMsg;

uint8_t throttle1Pin = A7;
uint8_t throttle2Pin = A6;
uint8_t brakePin = A5;
uint8_t tsSwitchPin = 5;
uint8_t pushButtonPin = 6;

MCP2515 mcp2515;

struct sensorValues {
  uint16_t throttle1Value;
  uint16_t throttle2Value;
  uint16_t brakeValue;
  uint8_t tsValue;
  bool keyValue;
};

void throttle1(void *pvParameters);
void throttle2(void *pvParameters);
void brake(void *pvParameters);
void send(void *pvParameters);
void ts(void *pvParameters);
void key(void *pvParameters);

sensorValues sensorVals{0, 0, 0, 0, 0};


void setup()
{
  Serial.begin(9600);
  SPI.begin();

  mcp2515.init(10); // CS pin as 10

  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS);
  mcp2515.setNormalMode();

  xTaskCreate(throttle1, "Pot 1", 100, nullptr, 1, nullptr);
  xTaskCreate(throttle2, "Pot 2", 100, nullptr, 1, nullptr);
  xTaskCreate(brake, "Brake", 100, nullptr, 1, nullptr);
  xTaskCreate(send, "Send", 100, nullptr, 1, nullptr);
  xTaskCreate(ts, "TS Switch", 100, nullptr, 1, nullptr);
  xTaskCreate(key, "Key Switch", 100, nullptr, 1, nullptr);
  vTaskStartScheduler();

  pinMode(throttle1Pin, INPUT);
  pinMode(throttle2Pin, INPUT);
  pinMode(tsSwitchPin, INPUT);
  pinMode(pushButtonPin, INPUT);

}

void loop() {}

void throttle1(void *pvParameters)
{
  while (true)
  {
    sensorVals.throttle1Value = analogRead(throttle1Pin);
  }
}

void throttle2(void *pvParameters)
{
  while (true)
  {
    sensorVals.throttle2Value = analogRead(throttle2Pin);
  }
}

void brake(void *pvParameteres)
{
  while (true)
  {
    sensorVals.brakeValue = analogRead(brakePin);
  }
}

void ts(void *pvParameteres)
{
  while (true)
  {
    sensorVals.tsValue = digitalRead(tsSwitchPin);
  }
}

void key(void *pvParameteres)
{

  int initialButtonState = LOW;
  int currentButtonState = HIGH;
  int buttonState = LOW;
  bool buttonReleased = true;    //Add a flag to track if the button has been released
  unsigned long lastDebounceTime = 0;
  unsigned long debounceDelay = 50;  //500 ms = .5 seconds

  while (true)
  {

    int reading = digitalRead(pushButtonPin);
    //reading = high

    if (reading != initialButtonState)  //if the current state is high
    {
      lastDebounceTime = millis();    //reset the debouncing timer
    }

//whatever the current state is, if the time since the last debounce is greater than the debounce delay
    if ((millis() - lastDebounceTime) > debounceDelay)
    {
      //if the reading is different than the button state
      if (reading != buttonState)
      {
        buttonState = reading;    //set the current reading to buttonState

        if (buttonState == HIGH && buttonReleased == true) {
          sensorVals.keyValue = !sensorVals.keyValue;
          buttonReleased = false; //reset the flag
        }
      }
      if (buttonState == LOW) {
        buttonReleased = true; //set the flag
      }
  }
  initialButtonState = reading;
}

void send(void *pvParameters)
{
  while (true)
  {
    canMsg.can_id = 0x036; // CAN id as 0x036
    canMsg.can_dlc = 8;    // CAN data length as 8
    canMsg.data[0] = sensorVals.throttle1Value >> 8;
    canMsg.data[1] = sensorVals.throttle1Value & 0xFF;
    canMsg.data[2] = sensorVals.throttle2Value >> 8;
    canMsg.data[3] = sensorVals.throttle2Value & 0xFF;
    canMsg.data[4] = sensorVals.brakeValue >> 8;
    canMsg.data[5] = sensorVals.brakeValue & 0xFF;
    canMsg.data[6] = sensorVals.tsValue;
    canMsg.data[7] = sensorVals.keyValue;

    mcp2515.sendMessage(&canMsg); // Sends the CAN message
    delay(10);

    Serial.print("tsValue: " );
    Serial.println(sensorVals.tsValue);
    Serial.print("keyValue: " );
    Serial.println(sensorVals.keyValue);
  }
}