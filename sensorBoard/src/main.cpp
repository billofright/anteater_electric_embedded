#include <Arduino.h>
#include <ChRt.h>
#include <FlexCAN_T4.h>

FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> sensorCAN;

uint8_t throttle1Pin = A7;
uint8_t throttle2Pin = A6;
uint8_t brakePin = A5;
uint8_t tsSwitchPin = 5;
uint8_t pushButtonPin = 6;

struct sensorValues
{
  uint16_t throttle1Value;
  uint16_t throttle2Value;
  uint16_t brakeValue;
  uint8_t tsValue;
  bool keyValue;
};

sensorValues sensorVals{0, 0, 0, 0, 0};

static THD_WORKING_AREA(waThread1, 64);
static THD_FUNCTION(throttle1, arg)
{
  (void)arg;
  while (true)
  {
    sensorVals.throttle1Value = analogRead(throttle1Pin);
  }
}

static THD_WORKING_AREA(waThread2, 64);
static THD_FUNCTION(throttle2, arg)
{
  (void)arg;
  while (true)
  {
    sensorVals.throttle2Value = analogRead(throttle2Pin);
  }
}

static THD_WORKING_AREA(waThread3, 64);
static THD_FUNCTION(brake, arg)
{
  (void)arg;
  while (true)
  {
    sensorVals.brakeValue = analogRead(brakePin);
  }
}

void canSniff(const CAN_message_t &msg)
{
  // Serial.print("MB "); Serial.print(msg.mb);
  // Serial.print("  OVERRUN: "); Serial.print(msg.flags.overrun);
  // Serial.print("  LEN: "); Serial.print(msg.len);
  // Serial.print(" EXT: "); Serial.print(msg.flags.extended);
  // Serial.print(" TS: "); Serial.print(msg.timestamp);
  // Serial.print(" ID: "); Serial.print(msg.id, HEX);
  // Serial.print(" Buffer: ");
  // for ( uint8_t i = 0; i < msg.len; i++ ) {
  //   Serial.print(msg.buf[i], HEX); Serial.print(" ");
  // } Serial.println();
  // delay(1000);
  // uint16_t rpm = (msg.buf[1] << 8) | msg.buf[0];
  // Serial.print("RPM : ");
  // Serial.println(rpm);

  // uint16_t current = ((msg.buf[3] << 8) | msg.buf[2]) /10;
  // Serial.print("Current : ");
  // Serial.println(current);

  // uint16_t voltage = ((msg.buf[5] << 8) | msg.buf[4]) /10;
  // Serial.print("Voltage : ");
  // Serial.println(voltage);

  // uint16_t error = (msg.buf[7] << 8) | msg.buf[6];
  // Serial.print("Error : ");
  // Serial.println(error);

  // message 2
  float throttleSignal = 5.0 / 255 * msg.buf[0];
  uint16_t controlTemp = msg.buf[1] - 40;
  uint16_t motorTemp = msg.buf[2] - 30;

  Serial.print("TS : ");
  Serial.print(throttleSignal);
  Serial.print("      ");
  Serial.println(msg.buf[0]);
  delay(100);
  Serial.print("CT : ");
  Serial.print(controlTemp);
  Serial.print("       MT : ");
  Serial.println(motorTemp);
}

static THD_WORKING_AREA(waThread4, 64);
static THD_FUNCTION(send, arg)
{
  (void)arg;
  CAN_message_t msg;
  while (true)
  {

    // msg.id = 0x036;
    // msg.len = 8;
    // memcpy(msg.buf, &sensorVals, sizeof(sensorVals));
    // sensorCAN.write(msg);

    // delay(100);
    // Serial.print("Hello ");
    msg.flags.extended = 1;
    // msg.id = 0x0CF11E05; // message id 1
    msg.id = 0x0CF11F05; // message id 2 - throttle signal, controller temp, motor temp
    msg.len = 8;
    sensorCAN.write(msg);
    // Serial.print("After write ");
    sensorCAN.onReceive(canSniff);
    // Serial.print("After recieve ");
  }
}

static THD_WORKING_AREA(waThread5, 64);
static THD_FUNCTION(ts, arg)
{
  (void)arg;
  while (true)
  {
    sensorVals.tsValue = digitalRead(tsSwitchPin);
  }
}

static THD_WORKING_AREA(waThread6, 64);
static THD_FUNCTION(key, arg)
{
  (void)arg;
  uint32_t lastPressed = millis();
  uint8_t prevValue = LOW;
  uint16_t debounceDelay = 500;

  while (true)
  {
    int currValue = digitalRead(pushButtonPin);
    if (currValue == HIGH && prevValue == LOW && millis() - lastPressed > debounceDelay)
    {
      lastPressed = millis();
      sensorVals.keyValue = !sensorVals.keyValue;
    }
    prevValue = currValue;
  }
}

void chSetup()
{
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
  sensorCAN.enableFIFO();
  sensorCAN.enableFIFOInterrupt();
  // pinMode(throttle1Pin, INPUT);
  // pinMode(throttle2Pin, INPUT);
  // pinMode(tsSwitchPin, INPUT);
  // pinMode(pushButtonPin, INPUT);

  chBegin(&chSetup);
}

void loop() {}
