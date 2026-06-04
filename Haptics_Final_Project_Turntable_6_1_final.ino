/*
  LCD circuit:
   LCD RS pin to digital pin 12
   LCD Enable pin to digital pin 11
   LCD D4 pin to digital pin 5
   LCD D5 pin to digital pin 4
   LCD D6 pin to digital pin 3
   LCD D7 pin to digital pin 2
   LCD R/W pin to ground
   LCD VSS pin to ground
   LCD VCC pin to 5V
   10K resistor:
   ends to +5V and ground
   wiper to LCD VO pin (pin 3)
  https://docs.arduino.cc/learn/electronics/lcd-displays
*/

// include the LCD library code:
#include <LiquidCrystal.h>

// include SoftwareSerial to allow for second serial interface
#include <SoftwareSerial.h>
//SoftwareSerial Serial1(7, 6); // RX=6, TX=13

// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
const int rs = 13, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;// rs originally 12
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);


//initialize other variables
int whichDisplay = 0;
int totalDisplayStates = 5;
float setpoint;
float sensor = 0.0;
int buttonPin = 8;//originally 8
int reset_button =12;//originally 7
int PWMA1 = 9;
int PWMA2 = 10;
float fan1Out;
float fan2Out;
float midpoint = 125.0;
float setpointScalar = 1.0;
float previousSetpoint = 0.0;
float setpointTolerance = 15.0;


float controlEffort;
float kp = 11.11; //hard-coded, found by trial and error
float ki = 0.55;  //hard-coded, found by trial and error
float kd = 16.67; //hard-coded, found by trial and error
float kp_scalar; //adjustable by user with potentiometer (from 0-200%)
float kd_scalar; //adjustable by user with potentiometer (from 0-200%)
float ki_scalar; //adjustable by user with potentiometer (from 0-200%)
float pTerm = 0;
float iTerm = 0;
float dTerm = 0;
float error = 0;
float lastError = 0;


//IMU code from: https://www.udoo.org/forum/threads/retrieve-mpu6500-imu-sensor-data-with-arduino.38473/
#include "FastIMU.h"
#include <Wire.h>
#define IMU_ADDRESS 0x68    //Address of the main IMU
#define IMU_ADDRESS2 0x69    //Address of the input IMU
#define PERFORM_CALIBRATION //Comment out this line to skip calibration at start
MPU6500 IMU(Wire);               //Change to the name of any supported IMU!
MPU6500 IMU2(Wire);               //Change to the name of any supported IMU!
calData calib = { 0 };  //Calibration data
AccelData accelData;    //Sensor data
GyroData gyroData;
calData calib2 = { 0 };  //Calibration data
AccelData accelData2;    //Sensor data
GyroData gyroData2;


float gyro_angle = 0.0;
float gyro_rate = 0.0;
float previousTime = 0.0;
float dt = 0.0;
float gyro_deadband = 2.00;
float gyro_deadband2 = 0.90;
float gyro_delta = 0.0;
float gyro_delta_deadband = 0.10;
float gyro_delta_deadband2 = 0.10;
float gyro_angle2 = 0.0;
float gyro_rate2 = 0.0;
float gyro_delta2 = 0.0;


void setup() {

  Serial.begin(115200);           //  setup serial
  Serial.setTimeout(0);
  //Serial.println("Write anything test!");

  //Serial1.begin(115200); // setup new serial for communicating with second Arduino
  //Serial1.setTimeout(0);

  pinMode(rs, OUTPUT);
  pinMode(en, OUTPUT);
  pinMode(d4, OUTPUT);
  pinMode(d5, OUTPUT);
  pinMode(d6, OUTPUT);
  pinMode(d7, OUTPUT);

  pinMode(buttonPin, INPUT);
  pinMode(reset_button, INPUT);

  analogWrite(PWMA1, 0); //turn off fans during calibration
  analogWrite(PWMA2, 0); //turn off fans during calibration


  lcd.begin(16, 2);  // set up the LCD's number of columns and rows
  lcd.setCursor(0, 0);
  lcd.print("Starting...");  // Print initial message to the LCD
  lcd.setCursor(0, 1);
  lcd.print("6/1 V10");
  delay(1000);


  Wire.begin();
  Wire.setClock(400000); //400khz clock


  int err = IMU.init(calib, IMU_ADDRESS);
  if (err != 0) {
    Serial.print("Error initializing IMU: ");
    Serial.println(err);
    while (true) {
      ;
    }
  }
  int err2 = IMU2.init(calib2, IMU_ADDRESS2);
  if (err2 != 0) {
    Serial.print("Error initializing IMU: ");
    Serial.println(err2);
    while (true) {
      ;
    }
  }


#ifdef PERFORM_CALIBRATION
  Serial.println("Calibrating main IMU");
  lcd.setCursor(0, 0);
  lcd.print("Calibrating 1/2");
  lcd.setCursor(0, 1);
  lcd.print("No touchy!");
  delay(250);
  Serial.println("Keep IMU level.");
  delay(5000);
  IMU.calibrateAccelGyro(&calib);
  Serial.println("Calibration done!");
  Serial.println("Accel biases X/Y/Z: ");
  Serial.print(calib.accelBias[0]);
  Serial.print(", ");
  Serial.print(calib.accelBias[1]);
  Serial.print(", ");
  Serial.println(calib.accelBias[2]);
  Serial.println("Gyro biases X/Y/Z: ");
  Serial.print(calib.gyroBias[0]);
  Serial.print(", ");
  Serial.print(calib.gyroBias[1]);
  Serial.print(", ");
  Serial.println(calib.gyroBias[2]);
  delay(1000);
  IMU.init(calib, IMU_ADDRESS);


  Serial.println("Calibrating input IMU");
  lcd.setCursor(0, 0);
  lcd.print("Calibrating 2/2");
  lcd.setCursor(0, 1);
  lcd.print("No touchy!");
  delay(250);
  Serial.println("Keep IMU level.");
  delay(5000);
  IMU2.calibrateAccelGyro(&calib2);
  Serial.println("Calibration done!");
  Serial.println("Accel biases X/Y/Z: ");
  Serial.print(calib2.accelBias[0]);
  Serial.print(", ");
  Serial.print(calib2.accelBias[1]);
  Serial.print(", ");
  Serial.println(calib2.accelBias[2]);
  Serial.println("Gyro biases X/Y/Z: ");
  Serial.print(calib2.gyroBias[0]);
  Serial.print(", ");
  Serial.print(calib2.gyroBias[1]);
  Serial.print(", ");
  Serial.println(calib2.gyroBias[2]);
  delay(1000);
  IMU2.init(calib2, IMU_ADDRESS2);
#endif


  //err = IMU.setGyroRange(500);      //USE THESE TO SET THE RANGE, IF AN INVALID RANGE IS SET IT WILL RETURN -1
  //err = IMU.setAccelRange(2);       //THESE TWO SET THE GYRO RANGE TO Â±500 DPS AND THE ACCELEROMETER RANGE TO Â±2g

  if (err != 0) {
    Serial.print("Error Setting range: ");
    Serial.println(err);
    while (true) {
      ;
    }
  }


  //err2 = IMU.setGyroRange(500);      //USE THESE TO SET THE RANGE, IF AN INVALID RANGE IS SET IT WILL RETURN -1
  //err2 = IMU.setAccelRange(2);       //THESE TWO SET THE GYRO RANGE TO Â±500 DPS AND THE ACCELEROMETER RANGE TO Â±2g

  if (err2 != 0) {
    Serial.print("Error Setting range: ");
    Serial.println(err2);
    while (true) {
      ;
    }
  }

}


void loop() {

  //Serial.println("In the main loop");

  //record loop time
  dt = (millis() - previousTime) / 1000;
  previousTime = millis();

  //read main IMU
  IMU.update();
  IMU.getAccel(&accelData);
  IMU.getGyro(&gyroData);
  gyro_rate = gyroData.gyroZ;
  //deadband
  if (abs(gyro_rate) < gyro_deadband) {
    gyro_rate = 0.0;
  }
  gyro_delta = gyro_rate * .028125;// scalar determined by trial and error to give good degree measurements
  if (abs(gyro_delta) < gyro_delta_deadband) {
    gyro_delta = 0.0;
  }
  gyro_angle = gyro_angle + gyro_delta;

  //read input gyro
  IMU2.update();
  IMU2.getAccel(&accelData2);
  IMU2.getGyro(&gyroData2);
  gyro_rate2 = gyroData2.gyroZ;
  //deadband
  if (abs(gyro_rate2) < gyro_deadband2) {
    gyro_rate2 = 0.0;
  }
  gyro_delta2 = gyro_rate2 * .225 - .08625 * dt; // scalar and drift correction determined by trial and error to give good degree measurements
  if (abs(gyro_delta2) < gyro_delta_deadband) {
    gyro_delta2 = 0.0;
  }
  gyro_angle2 = gyro_angle2 - gyro_delta2;
  //Serial.println("gyro rate: " + String(gyro_rate) + " gyro delta: " + String(gyro_delta) + " gyro angle: " + String(gyro_angle));
  //Serial.println("gyro rate: " + String(gyro_rate) + " gyro rate2: " + String(gyro_rate2));

// reset button to reset the platform angle and accumulated I term contribution
  if (digitalRead(reset_button) == HIGH) {
    gyro_angle = 0.0;
    iTerm = 0.0;
  }

// if the setpoint changes enough, reset the accumulated I term contribution
  if (abs(setpoint - previousSetpoint) > setpointTolerance){
    iTerm = 0.0;
    Serial.print("i reset triggered!");
  }
  previousSetpoint = setpoint;

  // //read second serial monitor for setpoint value
  // if (Serial1.available())
  // {
  //   delay(1);
  //     setpoint = Serial1.parseFloat();// * setpointScalar;
  //     while(Serial1.available() > 0 && (Serial1.peek() == '\n' || Serial1.peek() == '\r')) {
  //       Serial1.read();
  //   }
  // }
    //read second serial monitor for setpoint value
  if (Serial.available())
  {
    delay(1);
      setpoint = Serial.parseFloat();// * setpointScalar;
      while(Serial.available() > 0 && (Serial.peek() == '\n' || Serial.peek() == '\r')) {
        Serial.read();
    }
  }

  //setpoint = gyro_angle2; // overwritten by serial read above
  sensor = gyro_angle;

  //Serial1.println(sensor); // write out the current gyro angle to the other Arduino
  Serial.println(sensor); //test on the existing serial line (remove once verified)
  

  //read gain values
  kp_scalar = analogRead(A0) * 0.002; //scales from 0 to 2 (0 to 200%)
  kd_scalar = analogRead(A1) * 0.002;
  ki_scalar = analogRead(A2) * 0.002;


  //CONTROL LAW:


  //compute P, I , D terms
  error = (setpoint - sensor);
  pTerm = kp * kp_scalar * error;
  iTerm = iTerm + ki * ki_scalar * error * dt;
  dTerm = -1 * (kd * kd_scalar * (lastError - error) / dt);
  lastError = error;


  //sum P, I, D terms
  controlEffort = pTerm + iTerm + dTerm;


  //Outputs
  fan2Out = midpoint + controlEffort;
  fan1Out = midpoint - controlEffort;


  //clip fan PWM at 0 and 250 bounds
  if (fan1Out < 0)
  {
    fan1Out = 0;
  }
  else if (fan1Out > 250)
  {
    fan1Out = 250;
  }
  if (fan2Out < 0)
  {
    fan2Out = 0;
  }
  else if (fan2Out > 250)
  {
    fan2Out = 250;
  }

  fan1Out = fan1Out * 1.0;
  fan2Out = fan2Out * 1.0;


  //Serial.println("controlEffort: " + String(controlEffort) + " f1: " + String(fan1Out) + " f2: " + String(fan2Out));


  //state machine for using button to cycle display mode
  if (digitalRead (buttonPin) == HIGH) //when the button is pushed...
  {
    whichDisplay = whichDisplay + 1; //...incremement the display state...
    lcd.clear(); //...and clear the display.
    if (whichDisplay > totalDisplayStates - 1) //if the display state passes the last one...
    {
      whichDisplay = 0; //...wrap it back around to the first.
    }
  }
  switch (whichDisplay) {
    case 0:
      lcd.setCursor(0, 0);
      lcd.print("Setpoint :" + String(setpoint) +  "      ");
      lcd.setCursor(0, 1);
      //lcd.print("Sensor :" + String((sensor - .5)*180) + "      ");
      lcd.print("Sensor :" + String(sensor) + "      ");
      break;
    case 1:
      lcd.setCursor(0, 0);
      lcd.print("p: " + String(pTerm) + " i: " + String(iTerm));
      lcd.setCursor(0, 1);
      lcd.print("d: " + String(dTerm) + "      ");
      break;
    case 2:
      lcd.setCursor(0, 0);
      lcd.print("F1: " + String(fan1Out) + "      ");
      lcd.setCursor(0, 1);
      lcd.print("F2: " + String(fan2Out) + "      ");
      break;
    case 3:
      lcd.setCursor(0, 0);
      lcd.print("gRate: " + String(gyro_rate) + "      ");
      lcd.setCursor(0, 1);
      lcd.print("gAngle: " + String(gyro_angle) + "      ");
      break;
    case 4:
      lcd.setCursor(0, 0);
      lcd.print("Kp:" + String(round(kp_scalar * 100)) + "%" + " Kd:" + String(round(kd_scalar * 100)) + "% ");
      lcd.setCursor(0, 1);
      lcd.print("Ki:" + String(round(ki_scalar * 100)) + "%      ");
      break;
    default: // error handling
      lcd.setCursor(0, 0);
      lcd.print("Error:");
      lcd.setCursor(0, 1);
      lcd.print("Bad case");
      break;
  }


  //write the final outputs on the analog pins
  analogWrite(PWMA1, fan1Out);
  analogWrite(PWMA2, fan2Out);

  // delay so that the LCD updates slow enough to read without flashing
  delay(20); //originally 200
}
