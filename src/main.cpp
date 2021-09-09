#include <Arduino.h>

// this sketch is adapted from the following example https://github.com/jrowberg/i2cdevlib/blob/master/Arduino/MPU6050/examples/MPU6050_DMP6_using_DMP_V6.12/MPU6050_DMP6_using_DMP_V6.12.ino

struct Payload // MAX 32 BYTES
{
  // 12 bytes of MPU data
  int16_t ax, ay, az; // acceleration values in all 3 axis (-32768 to 32767)
  int16_t gx, gy, gz; // gyroscope acceleration values in all 3 axis (-32768 to 32767)

  float pressure; // 4 bytes
  int altitude;   // 2 bytes
  float temp;     // 4 bytes

  unsigned int vcc;       // 2 bytes
  unsigned int packet_id; // 2 bytes

  // we have 26 bytes total
};

Payload payload;

#include "nRF24L01.h" //NRF24L01 library created by TMRh20 https://github.com/TMRh20/RF24
#include "RF24.h"
#include "SPI.h"

RF24 radio(7, 8); // using SPI pins 7 and 8

// Single radio pipe address for the 2 nodes to communicate.
const uint64_t pipe = 0xE8E8F0F0E1LL;

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps_V6_12.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

#include "SparkFunBME280.h"
BME280 temp_sensor;
; // BMP280 Pressure sensor module library

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

void setup()
{
  // initialize serial communication
  Serial.begin(9600);

  Serial.print("Payload size: ");
  Serial.println(int(sizeof(payload)));

  Serial.println("Setting up radio");

  radio.begin();
  Serial.println("Setting radio data rate");
  radio.setDataRate(RF24_250KBPS);
  Serial.println("Setting radio power");
  radio.setPALevel(RF24_PA_MAX);
  Serial.println("Opening writing pipe");
  radio.openWritingPipe(pipe);

  Serial.println("Joining i2c bus");

// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.println("Finished joining i2c bus");

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  /*
  0 = +/- 2g
  1 = +/- 4g
  2 = +/- 8g
  3 = +/- 16g
  */
  mpu.setFullScaleAccelRange(3);

  /*
  0 = +/- 250 degrees/sec
  1 = +/- 500 degrees/sec
  2 = +/- 1000 degrees/sec
  3 = +/- 2000 degrees/sec
  */
  mpu.setFullScaleGyroRange(3);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  temp_sensor.setI2CAddress(0x76);
  temp_sensor.beginI2C();

  temp_sensor.setReferencePressure(102000); //Adjust the sea level pressure used for altitude calculations

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
}

unsigned int readVcc()
{
// https://provideyourown.com/2012/secret-arduino-voltmeter-measure-battery-voltage/
// Read 1.1V reference against AVcc
// set the reference to Vcc and the measurement to the internal 1.1V reference
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined(__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  ADMUX = _BV(MUX3) | _BV(MUX2);
#else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif

  delay(2);            // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA, ADSC))
    ; // measuring

  uint8_t low = ADCL;  // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both

  unsigned int result = (high << 8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result;              // Vcc in millivolts
}

long last_start_time = micros();
int packets_sent;
long timedelta;
float packets_per_second;
unsigned int packet_id;

void loop()
{
  mpu.getMotion6(
      &payload.ax, &payload.ay, &payload.az,
      &payload.gx, &payload.gy, &payload.gz
  );
  
  payload.pressure = temp_sensor.readFloatPressure();

  payload.altitude = temp_sensor.readFloatAltitudeMeters();

  payload.temp = temp_sensor.readTempC();

  ++packet_id;
  payload.packet_id = packet_id;

  radio.write(&payload, sizeof(Payload));

  ++packets_sent;

  timedelta = micros() - last_start_time;
  if (timedelta > 1000000)
  {
    packets_per_second = packets_sent / (timedelta / 1000000.000);
    Serial.print("PPS: ");
    Serial.println(packets_per_second);
    last_start_time = micros();
    packets_sent = 0;

    payload.vcc = readVcc();
    //Serial.print("Packet ID: ");
    //Serial.println(payload.packet_id);

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }
}