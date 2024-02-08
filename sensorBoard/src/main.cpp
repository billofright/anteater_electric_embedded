#include <Arduino.h>
#include <ChRt.h>
#include <FlexCAN_T4.h>

// #include <SPI.h>
// #include <mcp2515.h>
// #include <Arduino_FreeRTOS.h>
// struct can_frame canMsg;
// MCP2515 mcp2515;

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> sensorCAN;

uint8_t throttle1Pin = A7;
uint8_t throttle2Pin = A6;
uint8_t brakePin = A5;
uint8_t tsSwitchPin = 5;
uint8_t pushButtonPin = 6;

struct sensorValues {
  uint16_t throttle1Value;
  uint16_t throttle2Value;
  uint16_t brakeValue;
  uint8_t tsValue;
  bool keyValue;
};

// void throttle1(void *pvParameters);
// void throttle2(void *pvParameters);
// void brake(void *pvParameters);
// void send(void *pvParameters);
// void ts(void *pvParameters);
// void key(void *pvParameters);

sensorValues sensorVals{0, 0, 0, 0, 0};

static THD_WORKING_AREA(waThread1, 64);
static THD_FUNCTION(throttle1, arg) {
  (void)arg;
  while (true) {
    sensorVals.throttle1Value = analogRead(throttle1Pin);
  }
}

static THD_WORKING_AREA(waThread2, 64);
static THD_FUNCTION(throttle2, arg) {
  (void)arg;
  while (true) {
    sensorVals.throttle2Value = analogRead(throttle2Pin);
  }
}

static THD_WORKING_AREA(waThread3, 64);
static THD_FUNCTION(brake, arg) {
  (void)arg;
  while (true) {
    sensorVals.brakeValue = analogRead(brakePin);
  }
}

static THD_WORKING_AREA(waThread4, 64);
static THD_FUNCTION(send, arg) {
  (void)arg;
  CAN_message_t msg;
  while (true) {
    msg.id = 0x036;
    msg.len = 8;
    memcpy(msg.buf, &sensorVals, sizeof(sensorVals));
    sensorCAN.write(msg);

    delay(100);
  }
}

static THD_WORKING_AREA(waThread5, 64);
static THD_FUNCTION(ts, arg) {
  (void)arg;
  while (true) {
    sensorVals.tsValue = digitalRead(tsSwitchPin);
  }
}

static THD_WORKING_AREA(waThread6, 64);
static THD_FUNCTION(key, arg) {
  (void)arg;
  uint32_t lastPressed = millis();
  uint8_t prevValue = LOW;
  uint16_t debounceDelay = 500;

  while (true)
  {
    int currValue = digitalRead(pushButtonPin);
    if(currValue == HIGH && prevValue == LOW && millis() - lastPressed > debounceDelay){
      lastPressed = millis();
      sensorVals.keyValue = !sensorVals.keyValue;
    }
    prevValue = currValue;
  }

}

void chSetup(){
  // chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, throttle1, NULL);
  // chThdCreateStatic(waThread2, sizeof(waThread2), NORMALPRIO, throttle2, NULL);
  // chThdCreateStatic(waThread3, sizeof(waThread3), NORMALPRIO, brake, NULL);
  chThdCreateStatic(waThread4, sizeof(waThread4), NORMALPRIO, send, NULL);
  // chThdCreateStatic(waThread5, sizeof(waThread5), NORMALPRIO, ts, NULL);
  // chThdCreateStatic(waThread6, sizeof(waThread6), NORMALPRIO, key, NULL);
}


void setup()
{
  Serial.begin(9600);

  // CANFD_timings_t config;
  // config.clock = CLK_24MHz;
  // config.baudrate = 1000000;
  // config.baudrateFD = 2000000;
  // config.propdelay = 190;
  // config.bus_length = 1;
  // config.sample = 70;
  // sensorCAN.setBaudRate(config);
  sensorCAN.begin();
  sensorCAN.setBaudRate(250000);
  // pinMode(throttle1Pin, INPUT);
  // pinMode(throttle2Pin, INPUT);
  // pinMode(tsSwitchPin, INPUT);
  // pinMode(pushButtonPin, INPUT);

  chBegin(&chSetup);

  // SPI.begin();

  // mcp2515.init(10); // CS pin as 10

  // mcp2515.reset();
  // mcp2515.setBitrate(CAN_125KBPS);
  // mcp2515.setNormalMode();

  // xTaskCreate(throttle1, "Pot 1", 100, nullptr, 1, nullptr);
  // xTaskCreate(throttle2, "Pot 2", 100, nullptr, 1, nullptr);
  // xTaskCreate(brake, "Brake", 100, nullptr, 1, nullptr);
  // xTaskCreate(send, "Send", 100, nullptr, 1, nullptr);
  // xTaskCreate(ts, "TS Switch", 100, nullptr, 1, nullptr);
  // xTaskCreate(key, "Key Switch", 100, nullptr, 1, nullptr);
  // vTaskStartScheduler();

}

void loop() {}

// void send(void *pvParameters)
// {
//   while (true)
//   {
//     // canMsg.can_id = 0x036; // CAN id as 0x036
//     // canMsg.can_dlc = 8;    // CAN data length as 8
//     // canMsg.data[0] = sensorVals.throttle1Value >> 8;
//     // canMsg.data[1] = sensorVals.throttle1Value & 0xFF;
//     // canMsg.data[2] = sensorVals.throttle2Value >> 8;
//     // canMsg.data[3] = sensorVals.throttle2Value & 0xFF;
//     // canMsg.data[4] = sensorVals.brakeValue >> 8;
//     // canMsg.data[5] = sensorVals.brakeValue & 0xFF;
//     // canMsg.data[6] = sensorVals.tsValue;
//     // canMsg.data[7] = sensorVals.keyValue;

//     // mcp2515.sendMessage(&canMsg); // Sends the CAN message
//     // delay(10);

//     // Serial.print("tsValue: " );
//     // Serial.println(sensorVals.tsValue);
//     // Serial.print("keyValue: " );
//     // Serial.println(sensorVals.keyValue);
//   }
// }