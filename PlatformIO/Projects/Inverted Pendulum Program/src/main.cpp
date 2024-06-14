#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#include <SPI.h>
#include <Adafruit_ADS1X15.h>
#include <SparkFun_TB6612.h>

//////// VARIABLES AND DECLARATIONS ////////

// Gyro Potentiometer Data Points
float f0 = 1657, f1 = 814, f2 = 38, g0 = 30, g1 = 0, g2 = -30;
// Position Potentiometer Data Points
float a0 = 1674, a1 = 937, a2 = 207, b0 = 100, b1 = 0, b2 = -100;
// Gyro Potentiometer PWM Data Points
float z0 = 1657, z1 = 814, z2 = 38, w0 = 199, w1 = 127.5, w2 = 0;
// Position Potentiometer PWM Data Points
float c0 = 1674, c1 = 937, c2 = 207, d0 = 199, d1 = 127.5, d2 = 0;
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
// // Calibrated Data Points for Gyro Potentiometer
float leftAvg, middleAvg, rightAvg;
// PWM
float gyrPWM;
// Motor Driver
int PWMB = 9;
int BIN1 = 8;
int BIN2 = 10;
// Serial Monitor Variable
String receivedData;

/// TEST ///
float testPWM;
float gyrValTEST = 10;


// Function Declaration:
float intpolCalibration(int x, float x0, float x1, float x2, float y0, float y1, float y2);
float calculateAverage(int *array, int length);
void onlineCalibration();
void myISR();
void readADC_Gyro();
void readADC_Pos();
float readSerial();


// Function Definitions:
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
  gyrADC = ads1015.readADC_SingleEnded(0);
  gyrPWM = intpolCalibration(gyrADC, f0, f1, f2, w0, w1, w2);
  if (gyrPWM <= 0) {
    gyrPWM = 0;
  }
  Serial.println(gyrPWM);
  OCR1A = gyrPWM;
}

void readADC_Pos() {
  posADC = ads1015.readADC_SingleEnded(3);
}

float readSerial() {
  receivedData = Serial.readStringUntil('\n');
  testPWM = receivedData.toFloat();
  OCR1A = testPWM;
}

void setup() {
  // Setup Code

  // Start Serial Monitor
  Serial.begin(9600);
  // Serial.println("Hello World!");
  
  // Initialize ADS1015 
  ads1015.begin();

  // Set the gain (PGA)
  ads1015.setGain(GAIN_TWOTHIRDS); // +/- 6.144 V

  pinMode(13, OUTPUT); // Pin D13 used for LED to indicate interrupt state
  pinMode(2, INPUT); // Pin D2 used for Interrupt Flag
  pinMode(leftPos, INPUT); // Pin D3 used for left position calibration button
  pinMode(middlePos, INPUT); // Pin D4 used for middle position calibration button
  pinMode(rightPos, INPUT); // Pin D5 used for right position calibration button

  /// TEST ///
  pinMode(10, OUTPUT);

  //////// MOTOR DRIVER ////////
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  // Attach interrupt to Pin D2
  attachInterrupt(0, myISR, RISING);

  // FAST PWM 
  // Clear Timer1 control registers
  TCCR1A = 0;
  TCCR1B = 0;

  // Set Timer1 to Fast PWM mode with ICR1 as top value
  TCCR1A |= (1 << WGM11);
  TCCR1B |= (1 << WGM12) | (1 << WGM13);

  // Set non-inverting mode for PWM on OC1A (Pin 9)
  TCCR1A |= (1 << COM1A1);

  // Set the prescaler to 8 and start the timer
  TCCR1B |= (1 << CS11);

  // Set the PWM frequency by setting the ICR1 value
  ICR1 = 199;  // This sets the frequency to 10kHz

  // Set the initial duty cycle
  OCR1A = 0;  

}

void loop() {
  // Loop Code

  //////// PWM ////////
  readSerial();
  Serial.println(gyrValTEST);
  //////// ONLINE CALIBRATION ////////
  // 1. Use switch to enter calibration mode (interupt program) (use light to show that in calibration mode)
  // 2. Wait until one of the position buttons are pressed
  // 3. Read Gyro ADC values for the respective that is pressed (for the duration that it is pressed)
  // 4. Take average of all adc value and update calibration data points
  // 5. Exit calibration mode
  
  // Check if enableInt has been set
  if (enableInt) {
    onlineCalibration();
    f0 = rightAvg;
    f1 = middleAvg;
    f2 = leftAvg;
  }
  
  //////// ADC ////////

  // ADS1015 ADC reading for Gyro Potentiometer
  // readADC_Gyro();
  // readADC_Pos();
  // float gyrVolt = ads1015.computeVolts(gyrADC);
  // float posVolt = ads1015.computeVolts(posADC);

  // Calculate angle reading for Gyro Potentiometer using 3 point interpolation
  float gyrAngle = intpolCalibration(gyrADC, f0, f1, f2, g0, g1, g2);

  // Calculate distance reading for Position Potentiometer using 3 point interpolation
  float posDist = intpolCalibration(posADC, a0, a1, a2, b0, b1, b2);

  //////// MOTOR CONTROL ////////
  


  //////// OUTPUT SERIAL MONITOR ////////
  // Serial.println("----------------------------------");
  // Serial.print("Gyro ADC: ");
  // Serial.print(gyrADC);
  // Serial.print(", ");
  // Serial.print("Angle: ");
  // Serial.println(gyrAngle);
  // Serial.print("Position ADC: ");
  // Serial.print(posADC);
  // Serial.print(", ");
  // Serial.print("Distance: ");
  // Serial.println(posDist);



  //////// ARDUINO & PYTHON COMMUNICATION ////////

  // // Check if the serial monitor contains any data from python
  // if (Serial.available() > 0) { 
  //   // Parse information from python
  //   int interpolateVal = Serial.parseFloat(); 
  //   // Output information back to the serial monitor 
  //   Serial.print("Modified Value: ");
  //   Serial.println(interpolateVal);
  // }

  // Delay read cycle by 1000 milliseconds
  delay(1000);

}


