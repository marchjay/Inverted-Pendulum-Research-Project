#include <Arduino.h>
double gyrAngle = 26.4;
double posDist = 20;
String interpolateVal;
double pwmValue = 0;
void setup() {
    Serial.begin(9600); // Start the serial communication at 9600 baud rate
    pinMode(13, OUTPUT);
}

void loop() {

    

    Serial.println(gyrAngle);
    Serial.println(posDist);
    Serial.println(pwmValue);

    if (Serial.available() > 0) { 
        // Parse information from python
        digitalWrite(13, HIGH);
        interpolateVal = Serial.readStringUntil('\n');
        pwmValue = interpolateVal.toFloat();
        // Output information back to the serial monitor     
      }
    
    delay(1000); // Wait for a second
    
}


