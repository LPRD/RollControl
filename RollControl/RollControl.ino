// Code for controlling rocket roll during flight via 2 counter-rotating fins powered by a servo
// April 2017, Alex Bock

#define flying 1                // Change to 1 before flight    !!!!!!!!!!!!!!!!!!!!!!!!

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

int     rollTarget = 0;         // desired angular position
#define rollTol    0.00         
#define Kp  0.65
#define Kd -0.26
#define wc  3.927

#define controlTime1  4000      // time to execute roll maneuver (ms after launch detected)
#define controlTime2  7000
#define controlTime3  10000
#define controlPos1   90        // angle to maneuver to (relative to launch orientation)
#define controlPos2   180
#define controlPos3   270
#define launchAccel   38        // m/s^2 acceleration threshold to recognize launch has occured (39 is highest allowed)
long    launchTime = 0;
long    timeAfterLaunch = 0;
int     accel;

Adafruit_BNO055 bno = Adafruit_BNO055(55);
Servo servo1;
Servo servo2;
RTC_DS1307 RTC;
File dataFile;
char filename[] = "DATA000.csv";

#define LED           10
#define servo1Pin     3
#define servo2Pin     2         // current setup
#define data_size     3         // uses 0 (data_size-1 valid data points)
#define loopDelay     100       // min 200 to avoid missing deadlines (100 is fairly stable)
#define SDdelay       10
#define flagIncrement 10
#define sdErrorLimit  2
#define saveInterval  3000
#define dataTime ((float)loopDelay)/1000      // time between data
float eulerNew,eulerOld;
int i, j;
int flag = 0;
long checkSD,saveFlag,sheduleErrorTime;

// Offsets to make servos align vertically at v=90
#define servo1Offset 4          // for MG995 #1
#define servo2Offset 0          // for MG995 #2
#define vMax 14                 // max angular deflection (avoids stall)
int v = 90;                     // angle set to servo
int cross180 = -1;              // fixes control errors near 0/360 deg
float rollProp, rollDer;
float delta,deltaOld;

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


void setup() {
  
  if (flying) { pinMode(LED,OUTPUT); }              //  makes LED flash brightly
  Serial.begin(38400, SERIAL_8N2);    Serial.println();
  servo1.attach(3);
  servo2.attach(2);
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
        dataFile.println(F("abs time,sys date,sys time,servo_angle,x_euler_angle,y_euler_angle,z_euler_angle,x_gyro,y_gyro,z_gyro,"));
        break;
      }
    }
  }
  
}


void loop() {
  
  // Waits for launch to be detected
  if ((launchTime==0)&&(flying)) {
    while(accel<launchAccel) {
      imu::Vector<3> accelerometer = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
      accel = accelerometer.z();
      //Serial.print(accel); Serial.print("  /  "); Serial.println(launchAccel);
    }
    launchTime = millis();
  }
  
  timeAfterLaunch = millis() - launchTime;
  if      (timeAfterLaunch > controlTime3) {rollTarget = controlPos3;}
  else if (timeAfterLaunch > controlTime2) {rollTarget = controlPos2;}
  else if (timeAfterLaunch > controlTime1) {rollTarget = controlPos1;}
  //Serial.print("rollTarget   ");  Serial.println(rollTarget);
  
  long time0 = millis();
  eulerOld = eulerNew;
  imu::Vector<3> gyroscope     = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> euler         = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> accelerometer = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> magnetometer  = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  int8_t temp = bno.getTemp();
  eulerNew = euler.x();             // Depends on orientation of IMU with respect to rocket (x,y,z)
    
  // Checks if 180 deg has been crossed, fixes position errors for control
  //    an issue in this logic causes the "0" point to change
  if ((eulerNew>180)&&(eulerOld>180)&&(eulerNew>270)&&(eulerOld<270)) { 
    cross180 = 1;       //Serial.println("crossed CW");
  }
  if ((eulerNew<180)&&(eulerOld<180)&&(eulerNew<90)&&(eulerOld>90)) { 
    cross180 = -1;      //Serial.println("crossed CCW");
  }
  
  if ((eulerNew< 90)&&(cross180>0)) { eulerNew = eulerNew + 360; }
  if ((eulerNew>270)&&(cross180<0)) { eulerNew = eulerNew - 360; }
    
  // Proportional-Derivative Control
  rollProp =  eulerNew - rollTarget;
  rollDer  =  gyroscope.z();
  
  if (abs(rollProp)>=rollTol) {
    delta = Kp*rollProp + Kd*rollDer;
    //delta     = (1+wc*loopDelay)*deltaOld + wc*loopDelay*(Kp*rollProp + Kd*rollDer);    // adds rolloff
    deltaOld  = delta;                        // saves current value for next iteration
    if (delta>vMax)       { delta =  vMax; }
    else if (delta<-vMax) { delta = -vMax; }
    v = 90 + delta;
    //Serial.println(v);
    servo1.write(delta + 90 + servo1Offset);
    servo2.write(delta + 90 + servo2Offset);
  }
  
  // Downlink
  BEGIN_SEND
  SEND_ITEM(servo_angle, v);                        // 4 ms
  SEND_VECTOR_ITEM(euler_angle ,  euler);           // 18 ms
  SEND_VECTOR_ITEM(gyro        ,  gyroscope);       // 10 ms
  SEND_ITEM(temperature, temp);                     // 1 ms
  SEND_VECTOR_ITEM(magnetometer,  magnetometer);    // 10 ms
  SEND_VECTOR_ITEM(acceleration,  accelerometer);   // 18 ms
  END_SEND

  
  // Writing to SD Card
  // if() statements check for overly large time delays
  if ((flag<flagIncrement*sdErrorLimit-sdErrorLimit)&&(flag>0))   { flag--; }
  if ((flag<flagIncrement*sdErrorLimit-sdErrorLimit)&&(dataFile)) {
    DateTime now = RTC.now();           checkSD = millis();     // checks for SD removal
    dataFile.print(millis());           dataFile.print(',');
        if (millis()>checkSD+SDdelay){flag=flag+flagIncrement; goto timedout;}
    dataFile.print(now.year()  ,DEC);   dataFile.print('/');
    dataFile.print(now.month() ,DEC);   dataFile.print('/');
    dataFile.print(now.day()   ,DEC);   dataFile.print(',');
        if (millis()>checkSD+SDdelay){flag=flag+flagIncrement; goto timedout;}
    dataFile.print(now.hour()  ,DEC);   dataFile.print(':');
    dataFile.print(now.minute(),DEC);   dataFile.print(':');
    dataFile.print(now.second(),DEC);
        if (millis()>checkSD+SDdelay){flag=flag+flagIncrement; goto timedout;}
    WRITE_CSV_ITEM(v)
    WRITE_CSV_VECTOR_ITEM(euler)
        if (millis()>checkSD+SDdelay){flag=flag+flagIncrement; goto timedout;}
    WRITE_CSV_VECTOR_ITEM(gyroscope)
        
    timedout:
    dataFile.println();
    // saves to SD Card every [saveInterval] ms
    if (millis()>saveInterval*saveFlag+sheduleErrorTime) {
      dataFile.flush();
      saveFlag++;
    }
  }
  
  
  if (time0 + loopDelay < millis()) {
    Serial.print(F("Schedule err: "));
    Serial.println(time0 + loopDelay - (signed long)millis());
    sheduleErrorTime = sheduleErrorTime + (time0 + loopDelay - (signed long)millis());
    Serial.println(sheduleErrorTime);
    missed_deadlines++;
    SEND(missed_deadlines, missed_deadlines);
  }
  else {
    if(flag>=flagIncrement*sdErrorLimit-sdErrorLimit) { digitalWrite(LED,LOW); }
    delay(time0 + loopDelay - millis());    // continuously adjusted for desired dataTime
    digitalWrite(LED,HIGH);
  }
  
}
