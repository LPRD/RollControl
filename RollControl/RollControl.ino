#include <Wire.h>
#include <Servo.h>
#include <SPI.h>
#include <SD.h>
#include "RTClib.h"
#include "communications.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

float Kp =  2.8;
float Ki =  0.8;
float Kd = -0.3;
float rollTarget = 0.00;    // desired angular rotation
float rollTol = 0.00;

Adafruit_BNO055 bno = Adafruit_BNO055(55);
Servo servo1;
Servo servo2;
RTC_DS1307 RTC;
File dataFile;
char filename[] = "DATA000.csv";

#define servo1Pin  2
#define servo2Pin  3
#define loopDelay 200
#define dataTime ((float)loopDelay)/1000      // time between data
#define gyro_size 10        // uses 0 : gyro_size-1 valid data points
float gyro[gyro_size];
int i = 0;
int b;  int n;  int m;  int j;  int k;  int l;

// Offsets to make servos align vertically at exactly v=90
int servo1Offset = 4;       // 4 for MG995 #1
int servo2Offset = 0;       // 0 for MG995 #2
int v = 90;
int vMax = 12;              // max angular deflection (avoids stall)
float rollProp;
float rollInt;
float rollDer;


void setup() {
  pinMode(1,OUTPUT);
  servo1.attach(3);
  servo2.attach(2);
  Serial.begin(9600);
  Serial.println("");
  
  if (!bno.begin()) {
    Serial.println("BNO055 err");
    while (1);
  }
  
  if (! RTC.isrunning()) { RTC.adjust(DateTime(__DATE__, __TIME__)); }

  if (!SD.begin(10)) { Serial.println("SD err"); }
  
  for (uint16_t nameCount = 0; nameCount < 1000; nameCount++) {
    filename[4] = nameCount/100 + '0';
    filename[5] = (nameCount%100)/10 + '0';
    filename[6] = nameCount%10 + '0';
    if (!SD.exists(filename))     // only open if file doesn't exist
    {
      dataFile = SD.open(filename, FILE_WRITE);
      Serial.print("  writing to ");
      Serial.println(filename);
      break;
    }
  }
    
  Serial.print("\tv = ");
  Serial.println(v);
  servo1.write(v + servo1Offset);
  servo2.write(v + servo2Offset);
  delay(1000);
}



void loop() {
  long time0 = millis();
  i++;    if (i>=gyro_size)  { i = 0; }
  
  imu::Vector<3> gyroscope = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  gyro[i] = gyroscope.z();              // change dep. on orientation (x,y,z)
  
  Serial.print("gyro.z :  ");
  Serial.print(gyro[i]);
  Serial.println("");
  
  // Fixes reference errors with circular buffer
  b = i - 1;    if (b<1)  { b = b + gyro_size; }
  n = i - 2;    if (n<1)  { n = n + gyro_size; }
  j = i - 3;    if (j<1)  { j = j + gyro_size; }
  k = i - 4;    if (k<1)  { k = k + gyro_size; }
  l = i - 5;    if (l<1)  { l = l + gyro_size; }
  m = i - 6;    if (m<1)  { m = m + gyro_size; }
  
  // Proportional
  rollProp = (gyro[i]+gyro[b])/2 - rollTarget;
  
  // Integrated
  rollInt = (gyro[i]+gyro[n]+gyro[j]+gyro[k]+gyro[l]+gyro[m]-rollTarget*6) * dataTime*5;
  
  // Derivative       - just enough points to average instantaneous errors
  rollDer = ((gyro[i]-gyro[b])/(dataTime) + (gyro[i]-gyro[n])/(dataTime*2))/2;
  
  if (abs(rollProp)>=rollTol) {
        v = 90 + Kp*rollProp + Ki*rollInt + Kd*rollDer;
        
        if (v>90+vMax)      { v = 90 + vMax; }
        else if (v<90-vMax) { v = 90 - vMax; }
        
        servo1.write(v + servo1Offset);
        servo2.write(v + servo2Offset);
        
        // print angle components to serial
        Serial.print  ("  90");
        Serial.print  ("  +  ");
        Serial.print  (Kp*rollProp);
        Serial.print  ("  +  ");
        Serial.print  (Ki*rollInt);
        Serial.print  ("  +  ");
        Serial.print  (Kd*rollDer);
        Serial.print  ("  =  ");
        Serial.println(v);
  }
/*
  // Downlink
  sensors_event_t event; 
  bno.getEvent(&event);
  BEGIN_SEND
  SEND_ITEM(x_orientation, event.orientation.x);
  SEND_ITEM(y_orientation, event.orientation.x);
  SEND_ITEM(z_orientation, event.orientation.x);
  END_SEND
*/

  if (dataFile) {
    DateTime now = RTC.now();
    dataFile.print(now.year(), DEC);    dataFile.print('/');
    dataFile.print(now.month(), DEC);   dataFile.print('/');
    dataFile.print(now.day(), DEC);     dataFile.print(' ');
    dataFile.print(now.hour(), DEC);    dataFile.print(':');
    dataFile.print(now.minute(), DEC);  dataFile.print(':');
    dataFile.print(now.second(), DEC);  dataFile.print(", ");
    
    dataFile.print(gyro[i]);        dataFile.print(", ");
    dataFile.print(gyroscope.y());  dataFile.print(", ");
    dataFile.print(gyroscope.x());  dataFile.println("");
    dataFile.flush();
  }
  delay(time0 + loopDelay - millis());     // continuously adjusted for desired dataTime
}
