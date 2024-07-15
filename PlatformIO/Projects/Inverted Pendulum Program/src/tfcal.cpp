#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#include <SPI.h>
#include <Adafruit_ADS1X15.h>
#include <SparkFun_TB6612.h>

//////// VARIABLES AND DECLARATIONS ////////

// Gyro Potentiometer Data Points
float f0 = 1819.14, f1 = 948.08, f2 = 39.28, g0 = 30, g1 = 0, g2 = -30;
// Position Potentiometer Data Points
float a0 = 1653, a1 = 930, a2 = 202, b0 = 100, b1 = 0, b2 = -100;
// Gyro Potentiometer PWM Data Points
float z0 = 1819.14, z1 = 948.08, z2 = 39.28, w0 = 199, w1 = 127.5, w2 = 0;
// Position Potentiometer PWM Data Points
float c0 = 1653, c1 = 930, c2 = 202, d0 = 199, d1 = 127.5, d2 = 0;
// Online Calibration; Left, Middle, Right Button Pins
int leftPos = 3, middlePos = 4, rightPos = 5, index = 0;
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
int STBY = 11;
int offsetB = 1;

const int numSamples = 360;
int sineWave[numSamples];


// Function Declaration:
float intpolCalibration(int x, float x0, float x1, float x2, float y0, float y1, float y2);
void readADC_Gyro();
void readADC_Pos();
int mapFloat();

Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

// Function Definitions:
float intpolCalibration(int x, float x0, float x1, float x2, float y0, float y1, float y2) {
  float result;
  result = ((((x-x1)*(x-x2))/((x0-x1)*(x0-x2))) * y0) + ((((x-x0)*(x-x2))/((x1-x0)*(x1-x2))) * y1) + ((((x-x0)*(x-x1))/((x2-x0)*(x2-x1))) * y2);
  return result;
}

void readADC_Gyro() {
  gyrADC = ads1015.readADC_SingleEnded(1);
}

void readADC_Pos() {
  posADC = ads1015.readADC_SingleEnded(0);
}

int mapFloat(float x, float in_min, float in_max, int out_min, int out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
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

  //////// MOTOR DRIVER ////////
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

    for (int i = 0; i < numSamples; i++) {
        float angle = (2 * PI / numSamples) * i; // Angle in radians
        float sineValue = sin(angle);            // Sine of the angle
        int mappedValue = mapFloat(sineValue, -1.0, 1.0, -255, 255); // Map to -255 to 255
        sineWave[i] = mappedValue;               // Store in the array

        // Print the mapped value to the Serial Monitor for debugging
        Serial.println(mappedValue);

    }
}

void loop() {
  // Loop Code

  // Read Serial Monitor and send the calculated PWM value to the motor
//   readSerial();
    for (int i = 0; i < numSamples; i++) {
        int value = sineWave[i];
        motor2.drive(value);
    }
  // Check if enableInt has been set
//   if (enableInt) {
//     onlineCalibration();
//     f0 = rightAvg;
//     f1 = middleAvg;
//     f2 = leftAvg;
//   }
  
  //////// ADC ////////

  // ADS1015 ADC reading for Gyro Potentiometer
  readADC_Gyro();
  // float gyrVolt = ads1015.computeVolts(gyrADC);

  // ADS1015 ADC reading for Position Potentiometer
  readADC_Pos();
//   float posVolt = ads1015.computeVolts(posADC);

  // Calculate angle reading for Gyro Potentiometer using 3 point interpolation
  float gyrAngle = intpolCalibration(gyrADC, f0, f1, f2, g0, g1, g2);

  // Calculate distance reading for Position Potentiometer using 3 point interpolation
  float posDist = intpolCalibration(posADC, a0, a1, a2, b0, b1, b2);

  //////// OUTPUT SERIAL MONITOR ////////
  // Serial.println("----------------------------------");
  // Serial.print("Gyro ADC: ");
  // Serial.println(gyrADC);
  // Serial.print(", ");
  // Serial.print("Angle: ");
  Serial.println(gyrAngle);
  // Serial.print("Position ADC: ");
  // Serial.println(posADC);
  // Serial.print(", ");
  // Serial.print("Distance: ");
  Serial.println(posDist);

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
  delay(100);

}