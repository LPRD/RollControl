#include <SPI.h>
#include <SD.h>
#include "RTClib.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h>
#include "communications.h"

Servo servo1;
Servo servo2;
Adafruit_BNO055 bno = Adafruit_BNO055(55);

RTC_DS1307 RTC;
File dataFile;
char filename[] = "DATA000.csv";

float Kp = 3;
float Ki = 1.5;
float Kd = -0.1;

#define gyro_size 10
float gyro[gyro_size];            // 1 larger than intended buffer size (invalid data bug)
int i=0;
int b;  int n;  int m;  int j;  int k;  int l;
#define loopDelay 200
#define dataTime ((float)loopDelay)/1000      // time between data - approximated using millis()

float roll;                 // assumed axis of rotation (from last flight)
float rollTarget = 0;       // desired angular rotation
float rollTol = 0.00;
float rollProp;
float rollInt;
float rollDer;

// Offsets to make servos align vertically at exactly v=90
int servo1Offset = 4;       // 4 for MG995 #1
int servo2Offset = 0;       // 0 for MG995 #2
int v = 90;
int vMax = 12;              // max angular deflection (avoids stall)

void setup() {
  pinMode(1,OUTPUT);
  servo1.attach(3);
  servo2.attach(2);
  Serial.begin(9600);
  if (!bno.begin()) {
    Serial.println("BNO055 err");
    while (1);
  }

  /*Serial.print("Init SD ");
  if (!SD.begin(10)) {
    Serial.println("failed!");
    return;
  }
  //dataFile = SD.open("testing.txt", FILE_WRITE);
  for (uint16_t i = 0; i < 1000; i++) {
    filename[4] = i/100 + '0';
    filename[5] = (i%100)/10 + '0';
    filename[6] = i%10 + '0';
    if (!SD.exists(filename))     // only open a new file if it doesn't exist
    {
      dataFile = SD.open(filename, FILE_WRITE); 
      break;
    }
  }
  Serial.println("done!");*/
  
  Serial.println("\tReady");
  Serial.print("  v = ");
  Serial.println(v);
  servo1.write(v + servo1Offset);
  servo2.write(v + servo2Offset);
  delay(1000);
}

void loop() {
  long time0 = millis();
  i++;
  if (i>gyro_size)  { i = 0; }
  
  imu::Vector<3> gyroscope = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  gyro[i] = gyroscope.x();      //may need to change to y or z dep. on orientation
  
  Serial.print("\t\t\t\t\t\t\tX:  ");
  Serial.print(gyro[i]);
  Serial.println("");
  
  // Fixes reference errors with circular buffer
  b = i - 1;
    if (b<1)  { b = b+100; }
  n = i - 2;
    if (n<1)  { n = n+100; }
  j = i - 3;
    if (j<1)  { j = j+100; }
  k = i - 4;
    if (k<1)  { k = k+100; }
  l = i - 5;
    if (l<1)  { l = l+100; }
  m = i - 6;
    if (m<1)  { m = m+100; }
    
  // Proportional
  rollProp = (gyro[i]+gyro[b])/2 - rollTarget;

  // Integrated
  rollInt = (gyro[i]+gyro[n]+gyro[j]+gyro[k]+gyro[l]+gyro[m]-rollTarget*6) * dataTime*5;

  // Derivative       - just enough points to average instantaneous errors
  rollDer = ((gyro[i]-gyro[b])/(dataTime) + (gyro[i]-gyro[n])/(dataTime*2))/2;
  
  if (abs(rollProp)>=rollTol) {
        v = 90 + Kp*rollProp + Ki*rollInt + Kd*rollDer;
        
        if (v>90+vMax) {
            v = 90 + vMax;
        }
        else if (v<90-vMax) {
            v = 90 - vMax;
        }
        servo1.write(v + servo1Offset);
        servo2.write(v + servo2Offset);
        
        // prints angle components to serial
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

  // Downlink
  sensors_event_t event; 
  bno.getEvent(&event);
  BEGIN_SEND
  SEND_ITEM(x_orientation, event.orientation.x);
  SEND_ITEM(y_orientation, event.orientation.x);
  SEND_ITEM(z_orientation, event.orientation.x);
  END_SEND

  if (dataFile) {
    /*DateTime now = RTC.now();
    dataFile.print(now.year(), DEC);dataFile.print('/');
    dataFile.print(now.month(), DEC);dataFile.print('/');
    dataFile.print(now.day(), DEC);dataFile.print(' ');
    dataFile.print(now.hour(), DEC);dataFile.print(':');
    dataFile.print(now.minute(), DEC);dataFile.print('.');
    dataFile.print(now.second(), DEC);dataFile.print(", ");*/
    
    //Serial.print("Writing to file...");
    //dataFile.println(gyro[i]);
    Serial.println("done!");
  }
  /*dataFile.println(gyro[i]);*/
  
  delay(time0 + loopDelay - millis());           // adjusted for desired dataTime
}
