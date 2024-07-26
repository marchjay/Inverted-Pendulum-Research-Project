//////// JAY MARCH ////////

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#include <SPI.h>
#include <Adafruit_ADS1X15.h>
#include <SparkFun_TB6612.h>

//////// VARIABLES AND DECLARATIONS ////////

// Gyro Potentiometer Data Points
float f0 = 1817.20, f1 = 940.04, f2 = 16, g0 = 30, g1 = 0, g2 = -30;
// Position Potentiometer Data Points
float a0 = 1653, a1 = 930, a2 = 202, b0 = 30, b1 = 0, b2 = -30;
// Online Calibration; Left, Middle, Right Button Pins
int leftPos = 3, middlePos = 4, rightPos = 5, index = 0;
// Data Pool size for calibration
const int numSamples = 50;
// Arrays for calibration
int leftPosADC[numSamples], middlePosADC[numSamples], rightPosADC[numSamples]; 
// Array capacity flags
bool leftArrayFull = false, middleArrayFull = false, rightArrayFull = false;
// Interrupt Flag
volatile bool enableInt = false;
// ADS1015 Declaration
Adafruit_ADS1015 ads1015;
// Analog sensor variables (ADC)
int16_t gyrADC, posADC;
double gyrAngle;
// // Calibrated Data Points for Gyro Potentiometer
float leftAvg, middleAvg, rightAvg;
// Motor Driver
int PWMB = 9; // P
int BIN1 = 8; // Y
int BIN2 = 10; // W
int STBY = 11; // G
int offsetB = 1;
// Serial Monitor Variable
String receivedData;
// PID Controller Variables
double Kp = 80, Ki = 0, Kd = 40;
double Output, Setpoint = 0;
unsigned long currentTime, previousTime;
double elapsedTime;
double error, lastError, cumError, rateError;
double KpP = 10, KiP = 0.0, KdP = 1;
double OutputP, SetpointP = 0;
unsigned long currentTimeP, previousTimeP;
double elapsedTimeP;
double errorP, lastErrorP, cumErrorP, rateErrorP;
int i = 0;
//// -------- ////

//////// FUNCTION DECLARATIONS ////////

float intpolCalibration(int x, float x0, float x1, float x2, float y0, float y1, float y2);
float calculateAverage(int *array, int length);
void onlineCalibration();
void myISR();
void readADC_Gyro();
void readADC_Pos();
void readSerial();
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);
//// -------- ////

//////// FUNCTION DEFINITIONS ////////

float intpolCalibration(int x, float x0, float x1, float x2, float y0, float y1, float y2) {
  float result;
  result = ((((x-x1)*(x-x2))/((x0-x1)*(x0-x2))) * y0) + ((((x-x0)*(x-x2))/((x1-x0)*(x1-x2))) * y1) + ((((x-x0)*(x-x1))/((x2-x0)*(x2-x1))) * y2);
  return result;
}

float calculateAverage(int *array, int length) {
  long sum = 0;
  for (int i = 0; i < length; i++) {
    sum += array[i];
  }
  return (float)sum / (float)length;
}

void onlineCalibration() {

  digitalWrite(13, HIGH); // Turn on LED when in calibration mode
  Serial.println("PROGRAM INTERRUPTED FOR CALIBRATION");
  delay(1000);

  while (enableInt) {

    Serial.println("WAITING FOR CALIBRATION");
    
    if (not leftArrayFull) {
        while (digitalRead(leftPos) == 1) { // Left Position Calibration
        // Collect data
        readADC_Gyro();
        leftPosADC[index] = gyrADC;
        index++;
        Serial.println("MEASURING LEFT POSITION ADC");
        Serial.println(gyrADC);
        if (index >= numSamples) {
          digitalWrite(13, LOW);
          delay(100);
          digitalWrite(13,HIGH);
          index = 0;
          break;
        }
        delay(100);
      }
      leftAvg = calculateAverage(leftPosADC, numSamples);
      if (leftAvg > 0) {
      leftArrayFull = true; 
      }
    }

    else if (not middleArrayFull) {
        while (digitalRead(middlePos) == 1) { // Middle Position Calibration
        // Collect data
        readADC_Gyro();
        middlePosADC[index] = gyrADC;
        index++;
        Serial.println("MEASURING MIDDLE POSITION ADC");
        Serial.println(gyrADC);
        if (index >= numSamples) {
          digitalWrite(13, LOW);
          delay(100);
          digitalWrite(13,HIGH);
          index = 0;
          break;
        }
        delay(100);
      }
      middleAvg = calculateAverage(middlePosADC, numSamples);
      if (middleAvg > 0) {
      middleArrayFull = true;   
      }    
    }

    else if (not rightArrayFull) {
        while (digitalRead(rightPos) == 1) { // Right Position Calibration
        // Collect data
        readADC_Gyro();
        rightPosADC[index] = gyrADC;
        index++;
        Serial.println("MEASURING RIGHT POSITION ADC");
        Serial.println(gyrADC);
        if (index >= numSamples) {
          digitalWrite(13, LOW);
          delay(100);
          digitalWrite(13,HIGH);
          index = 0;
          break;
        }
        delay(100);
      }
      rightAvg = calculateAverage(rightPosADC, numSamples);
      if (rightAvg > 0) {
      rightArrayFull = true;
      }
    }

    else if (leftArrayFull & middleArrayFull & rightArrayFull) {
        enableInt = false;
    }
  }  

  Serial.print("Left Position Calibration: ");
  Serial.println(leftAvg);
  Serial.print("Middle Position Calibration: ");
  Serial.println(middleAvg);
  Serial.print("Right Position Calibration: ");
  Serial.println(rightAvg);
  delay(1000);
  Serial.println("YOU MAY NOW RESUME YOUR PROGRAM");
  delay(1000);
  digitalWrite(13, LOW);
}

void myISR() {
  // Enable Interrupt
  enableInt = true; 
}

void readADC_Gyro() {
  gyrADC = ads1015.readADC_SingleEnded(1);
}

void readADC_Pos() {
  posADC = ads1015.readADC_SingleEnded(0);
}

void readSerial() {
  receivedData = Serial.readStringUntil('\n');
  float gyrPWM = receivedData.toFloat();
}
//// -------- ////

void setup() {
  Serial.begin(250000);
  
  // Initialize ADS1015 
  ads1015.begin();
  ads1015.setGain(GAIN_TWOTHIRDS); // +/- 6.144 V

  //////// ONLINE CALIBRATION CONFIGURATION ////////

  pinMode(13, OUTPUT); // Pin D13 used for LED to indicate interrupt state
  pinMode(2, INPUT); // Pin D2 used for Interrupt Flag
  pinMode(leftPos, INPUT); // Pin D3 used for left position calibration button
  pinMode(middlePos, INPUT); // Pin D4 used for middle position calibration button
  pinMode(rightPos, INPUT); // Pin D5 used for right position calibration button
  //// -------- ////

  //////// MOTOR DRIVER ////////

  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  //// -------- ////

  // Attach interrupt to Pin D2
  attachInterrupt(0, myISR, RISING);
}

void loop() {
  // Check if enableInt has been set
  if (enableInt) {
    onlineCalibration();
    f0 = rightAvg;
    f1 = middleAvg;
    f2 = leftAvg;
  }
  
  //////// ADC ////////

  // ADS1015 ADC reading for Gyro Potentiometer
  readADC_Gyro();
  
  // ADS1015 ADC reading for Position Potentiometer
  readADC_Pos();
  // Calculate angle reading for Gyro Potentiometer using 3 point interpolation
  float gyrAngle = intpolCalibration(gyrADC, f0, f1, f2, g0, g1, g2);
  if (gyrAngle > -0.37 && gyrAngle < 0.37) {
    gyrAngle = 0;
  }
  // Calculate distance reading for Position Potentiometer using 3 point interpolation
  float posDist = intpolCalibration(posADC, a0, a1, a2, b0, b1, b2);
  float posVolt = ads1015.computeVolts(posADC);
  //// -------- ////

  //////// PID CONTROLLER ////////

      //// ANGLE PID CONTROLLER ////
  currentTime = millis();                               // Get current time
  elapsedTime = (double)(currentTime - previousTime);  // Calulcate the elapsed time, from previous loop
  
  error = Setpoint - gyrAngle;                           
  if (i >= 3) {
    cumError = 0;
    i = 0;
  } else {
    cumError += error * elapsedTime;    
    i = i + 1;                    
  }
  rateError = (error - lastError) / elapsedTime;         

  double output = Kp*error + Ki*cumError + Kd*rateError;                       
    //// -------- ////

  lastError = error;                                        // Store the previous error
  previousTime = currentTime;                              // Store the initial time measurement   

    //// POSITION PID CONTROLLER ////
  // currentTimeP = millis();                          
  // elapsedTimeP = (double)(currentTimeP - previousTimeP);     
  
  // errorP = SetpointP - posDist;                               
  // cumErrorP += errorP * elapsedTimeP;                    
  // rateErrorP = (errorP - lastErrorP) / elapsedTimeP;      

  // double outputP = KpP*errorP + KiP*cumErrorP + KdP*rateErrorP;  
  // outputP = outputP * -1;  
  // Serial.println(posDist); 
  
  
  // lastErrorP = errorP;                                    
  // previousTimeP = currentTimeP; 

    //// -------- ////

  // Serial.println(posDist);
  // Serial.println(gyrAngle);

  // if (posDist <= -20) {
  //   digitalWrite(13, HIGH);
  //   motor2.drive(255, 10);
  // } 

  // else if (posDist >= 20) {
  //   digitalWrite(13, HIGH);
  // } else {
  //   digitalWrite(13, LOW);
  //   motor2.drive(-255, 10);
  // }
  
  Output = output;
  Output = constrain(Output, -255, 255);
  //Serial.println(Output);
  motor2.drive(Output);

  

  ////// OUTPUT SERIAL MONITOR ////////

  delay(1);
}


