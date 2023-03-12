#include "TRSensors.h"

#define NUM_SENSORS 5

// sensors 0 through 4 are connected to analog inputs 0 through 5, respectively

TRSensors trs = TRSensors();
unsigned int sensorValues[NUM_SENSORS];

volatile int  lastError = 0;

volatile float Ki = 0.0000;
volatile float Kp = 0.035;
volatile float Kd = 0.47;

volatile int speedA = 100;
volatile int speedB = 100;

volatile int P;
volatile int I;
volatile int D;

void setup() {

  pinMode(A0, OUTPUT);
  pinMode(A1, OUTPUT);
  // avaliable left motor spining
  pinMode(5, OUTPUT);
  // Right motor
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  // avaliable right motor spining
  pinMode(6, OUTPUT);
  Serial.begin(115200);
  Serial.println("Sensors calibration...");

  for (int i = 0; i < 400; i++)  // make the calibration take about 10 seconds
  {
    trs.calibrate();  // reads all sensors 10 times
  }

  digitalWrite(A0,HIGH);
  analogWrite(5,200);
  delay(1000);
   digitalWrite(A0,LOW);
  digitalWrite(A3,HIGH);
  analogWrite(6,200);
  delay(1000);
 digitalWrite(A3,LOW);

  delay(4000);
  
 // Serial.println("calibrate done");
  // print the calibration minimum values measured when emitters were on
//  for (int i = 0; i < NUM_SENSORS; i++) {
//    Serial.print(trs.calibratedMin[i]);
//    Serial.print(' ');
//  }
//  Serial.println();
//
//  // print the calibration maximum values measured when emitters were on
//  for (int i = 0; i < NUM_SENSORS; i++) {
//    Serial.print(trs.calibratedMax[i]);
//    Serial.print(' ');
//  }
//  Serial.println();

  delay(1000);
  digitalWrite(A2, HIGH);
  digitalWrite(A1, HIGH);

  //digitalWrite(A3,LOW);
  //digitalWrite(A1, LOW);
}


void loop() {

  // read calibrated sensor values and obtain a measure of the line position from 0 to 5000
  // To get raw sensor values, call:
  // trs.AnalogRead(sensorValues);
  // trs.readCalibrated(sensorValues);

  unsigned int position = trs.readLine(sensorValues);

  //   print the sensor values as numbers from 0 to 1000, where 0 means maximum reflectance and
  //   1000 means minimum reflectance, followed by the line position
  //  for (unsigned char i = 0; i < NUM_SENSORS; i++)
  //  {
  //    Serial.print(sensorValues[i]);
  //    Serial.print('\t');
  //  }

  Serial.println(position);
  PID(position);
  delay(20);

  //     Serial.print(target-poprawka);
  //     Serial.println();
  //     Serial.print(target+poprawka);
  //     Serial.println();


  // delay(250);
}

void PID(int position) {
  
  if (position > 4001) {
    position = 4001;
  }

 int error = position-2000;

  P = error;

  I = I + error;

  D = (error - lastError)/50;

  lastError = error;

  int speedChange = Kp*P + Ki*I + Kd*D;

//int speedChange=Kp*P+Kd*D;

  Serial.print("Speed change:    ");
  Serial.println(speedChange);
  speedA = 100-speedChange;
  speedB = 100+speedChange;

  if (speedA > 255) {
    speedA = 255;
  }
  if (speedB > 255) {
    speedB = 255;
  }
  if (speedA < 0) {
    speedA = 0;
  }
  if (speedB < 0) {
    speedB = 0;
  }

  Serial.print("Speed A:    ");
  Serial.println(speedA);
  Serial.print("Speed B:    ");
  Serial.println(speedB);
  
  analogWrite(5,speedA);
  analogWrite(6,speedB);
}
