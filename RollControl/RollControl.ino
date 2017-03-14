// Code for controlling rocket roll during flight via 2 counter-rotating fins powered by a servo

#define flying 0                // Change to 1 before flight    !!!!!!!!!!!!!!!!!!!!!!!!
#include <Wire.h>
#include <Servo.h>
#include <SPI.h>
#include <SD.h>
#include <RTClib.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <avr/pgmspace.h>
#include <Telemetry.h>

#define rollTarget 0.00         // desired angular rotation
#define rollTol    0.00
#define Kp  1
#define Kd  0.5

Adafruit_BNO055 bno = Adafruit_BNO055(55);
Servo servo1;
Servo servo2;
RTC_DS1307 RTC;
File dataFile;
char filename[] = "DATA000.csv";

#define LED           10
#define servo1Pin     3
#define servo2Pin     2         // current setup
#define gyro_size     8         // uses 0 (gyro_size-1 valid data points)
#define loopDelay     200       // min 200
#define SDdelay       20
#define flagIncrement 10
#define sdErrorLimit  2
#define dataTime ((float)loopDelay)/1000      // time between data
float gyro[gyro_size];
int i, j, k, l, m, n, o;
int flag = 0;       long checkSD;

// Offsets to make servos align vertically at exactly v=90
#define servo1Offset 4          // for MG995 #1
#define servo2Offset 0          // for MG995 #2
#define vMax 12                 // max angular deflection (avoids stall)
int v = 90;
float rollProp, rollDer;

#define SEND_VECTOR_ITEM(field, value)\
  SEND_ITEM(field, value.x())         \
  SEND_GROUP_ITEM(value.y())          \
  SEND_GROUP_ITEM(value.z())

#define WRITE_CSV_ITEM(value)         \
  dataFile.print(F(", ")); dataFile.print(value);
  
#define WRITE_CSV_VECTOR_ITEM(value)  \
  WRITE_CSV_ITEM(value.x())           \
  WRITE_CSV_ITEM(value.y())           \
  WRITE_CSV_ITEM(value.z())

unsigned int missed_deadlines = 0;
long time1, time2;


void setup() {
  if (flying) { pinMode(LED,OUTPUT); }              //  makes LED flash brightly
  servo1.attach(3);
  servo2.attach(2);
  Serial.begin(38400, SERIAL_8N2);    Serial.println();
  servo1.write(v + servo1Offset);
  servo2.write(v + servo2Offset);
  
  while (!bno.begin()) {                            // flashes to signal error
    Serial.println(F("BNO055 err"));
    digitalWrite(LED,LOW); delay(1000); digitalWrite(LED,HIGH);
  }
  if (!RTC.isrunning()) { RTC.adjust(DateTime(__DATE__, __TIME__)); }
  if (!SD.begin(10)) { Serial.println(F("SD err")); }
  else {                                            // generates file name
    for (uint16_t nameCount = 0; nameCount < 1000; nameCount++) {
      filename[4] = nameCount/100 + '0';
      filename[5] = (nameCount%100)/10 + '0';
      filename[6] = nameCount%10 + '0';
      if (!SD.exists(filename)) {                   // opens if file doesn't exist
        dataFile = SD.open(filename, FILE_WRITE);
        Serial.print(F("\twriting "));
        Serial.println(filename);
        dataFile.println(F("abs time,sys date,sys time,temperature,servo_angle,x_gyro,y_gyro,z_gyro,x_magnetometer,y_magnetometer,z_magnetometer,x_euler_angle,y_euler_angle,z_euler_angle,x_acceleration,y_acceleration,z_acceleration"));
        break;
      }
    }
  }
  
}


void loop() {
  long time0 = millis();
  i++;    if (i>=gyro_size)  { i = 0; }
  
  imu::Vector<3> gyroscope     = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> euler         = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> accelerometer = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> magnetometer  = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  int8_t temp = bno.getTemp();
  gyro[i] = gyroscope.z();                          // change dep. on orientation of IMU (x,y,z)
  
  // Fixes references with circular buffer of recent data
  j = i - 1;    if (j<1)  { j = j + gyro_size; }
  k = i - 2;    if (k<1)  { k = k + gyro_size; }
  l = i - 3;    if (l<1)  { l = l + gyro_size; }
  m = i - 4;    if (m<1)  { m = m + gyro_size; }
  n = i - 5;    if (n<1)  { n = n + gyro_size; }
  o = i - 6;    if (o<1)  { o = o + gyro_size; }
  
  // Proportional-Derivative Control
  rollProp =  (gyro[i]+gyro[j])/2-rollTarget;
  rollDer  = ((gyro[i]-gyro[j])/(dataTime)+(gyro[i]-gyro[k])/(dataTime*2))/2;
  
  if (abs(rollProp)>=rollTol) {
        v = 90 + Kp*rollProp + Kd*rollDer;
        if (v>90+vMax)      { v = 90 + vMax; }
        else if (v<90-vMax) { v = 90 - vMax; }
        servo1.write(v + servo1Offset);
        servo2.write(v + servo2Offset);
  }
  
  // Downlink
  BEGIN_SEND
  SEND_ITEM(temperature, temp);                     // 1 ms
  SEND_ITEM(servo_angle, v);                        // 4 ms
  SEND_VECTOR_ITEM(gyro        ,  gyroscope);       // 10 ms
  SEND_VECTOR_ITEM(magnetometer,  magnetometer);    // 10 ms
  SEND_VECTOR_ITEM(euler_angle ,  euler);           // 18 ms
  SEND_VECTOR_ITEM(acceleration,  accelerometer);   // 18 ms
  END_SEND
  
  // Writing to SD Card
  if ((flag<flagIncrement*sdErrorLimit-sdErrorLimit)&&(flag>0))   { flag--; }
  if ((flag<flagIncrement*sdErrorLimit-sdErrorLimit)&&(dataFile)) {
    DateTime now = RTC.now();           checkSD = millis();     // checks for SD removal
    dataFile.print(millis());           dataFile.print(',');
        if (millis()>checkSD+SDdelay){flag=flag+flagIncrement; goto timedout;}
    dataFile.print(now.year()  ,DEC);   dataFile.print('/');
    dataFile.print(now.month() ,DEC);   dataFile.print('/');
        if (millis()>checkSD+SDdelay){flag=flag+flagIncrement; goto timedout;}
    dataFile.print(now.day()   ,DEC);   dataFile.print(',');
    dataFile.print(now.hour()  ,DEC);   dataFile.print(':');
        if (millis()>checkSD+SDdelay){flag=flag+flagIncrement; goto timedout;}
    dataFile.print(now.minute(),DEC);   dataFile.print(':');
    dataFile.print(now.second(),DEC);
        if (millis()>checkSD+SDdelay){flag=flag+flagIncrement; goto timedout;}
    WRITE_CSV_ITEM(temp)
    WRITE_CSV_VECTOR_ITEM(magnetometer)
        if (millis()>checkSD+SDdelay){flag=flag+flagIncrement; goto timedout;}
    WRITE_CSV_ITEM(v)
    WRITE_CSV_VECTOR_ITEM(gyroscope)
        if (millis()>checkSD+SDdelay){flag=flag+flagIncrement; goto timedout;}
    WRITE_CSV_VECTOR_ITEM(euler)
    WRITE_CSV_VECTOR_ITEM(accelerometer)
    timedout:
    dataFile.println();     dataFile.flush();
  }
  
  if (time0 + loopDelay < millis()) {
    Serial.print(F("Schedule err: "));
    Serial.println(time0 + loopDelay - (signed long)millis());
    missed_deadlines++;
    SEND(missed_deadlines, missed_deadlines);
  }
  else {
    if(flag>=flagIncrement*sdErrorLimit-sdErrorLimit) { digitalWrite(LED,LOW); }
    delay(time0 + loopDelay - millis());    // continuously adjusted for desired dataTime
    digitalWrite(LED,HIGH);
  }
  
}
