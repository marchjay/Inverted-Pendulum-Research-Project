#include <main.cpp>


// Function Definition
float onlineCalibration() {

  digitalWrite(13, HIGH); // Turn on LED when in calibration mode
  delay(1000);
  Serial.println("HELLO! YOU HAVE SUCCESFULLY INTERRUPTED THE PROGRAM");
  delay(1000);

  while (enableInt) {
    
    if (leftArrayFull = false) {
        while (digitalRead(leftPos) == 1) { // Left Position Calibration
        // Collect data
        readADC();
        leftPosADC[index] = adc_reading;
        index++;
        Serial.println("MEASURIIINNNGNGG COLLECTING LEFT");
        Serial.println(adc_reading);
        if (index >= numSamples) {
          index = 0;
          break;
        }
        delay(100);
      }
      float leftAvg = calculateAverage(leftPosADC, numSamples);
      Serial.println(leftAvg);
      leftArrayFull = true; 
    }

    else if (middleArrayFull = false) {
        while (digitalRead(middlePos) == 1) { // Middle Position Calibration
        // Collect data
        readADC();
        middlePosADC[index] = adc_reading;
        index++;
        Serial.println("MEASURIIINNNGNGG COLLECTING MIDDLE");
        Serial.println(adc_reading);
        if (index >= numSamples) {
          index = 0;
          break;
        }
        delay(100);
      }
      float middleAvg = calculateAverage(middlePosADC, numSamples);
      Serial.println(middleAvg);
      middleArrayFull = true;       
    }

    else if (rightArrayFull = false) {
        while (digitalRead(rightPos) == 1) { // Right Position Calibration
        // Collect data
        readADC();
        rightPosADC[index] = adc_reading;
        index++;
        Serial.println("MEASURIIINNNGNGG COLLECTING RIGHT");
        Serial.println(adc_reading);
        if (index >= numSamples) {
          index = 0;
          break;
        }
        delay(100);
      }
      float rightAvg = calculateAverage(rightPosADC, numSamples);
      Serial.println(rightAvg);
      rightArrayFull = true;
    }

    else if (leftArrayFull & middleArrayFull & rightArrayFull) {
        enableInt = false;
    }
  }  

  Serial.println("YOU MAY NOW RESUME YOUR PROGRAM");
  delay(1000);
  digitalWrite(13, LOW);
  
}
    
    
    
    
    
    
    
    
    
    
    
    
    
    
//     while (not leftArrayFull) {
//       float average = calculateAverage(leftPosADC, numSamples);
//       Serial.println(average);
//       if (average > 0) {
//         leftArrayFull = true;
//         break;
//       }
//       while (digitalRead(leftPos) == 1) { // Left Position Calibration
//         // Collect data
//         readADC();
//         leftPosADC[index] = adc_reading;
//         index++;
//         Serial.println("MEASURIIINNNGNGG COLLECTING");
//         Serial.println(adc_reading);
//         if (index >= numSamples) {
//           index = 0;
//           break;
//         }
//         delay(100);
//       }
      
     
//       delay(100);
//       // readADC();
//       // Serial.println(adc_reading);
//     } 

//   enableInt = false;   
//   } 

  Serial.println("YOU MAY NOW RESUME YOUR PROGRAM");
  delay(1000);
  digitalWrite(13, LOW);
  
}

float calculateAverage(int *array, int length) {
  long sum = 0;
  for (int i = 0; i < length; i++) {
    sum += array[i];
    
  }
  return (float)sum / (float)length;
  
}

void readADC() {
  adc_reading = ads1015.readADC_SingleEnded(0);
}



////// OLD STUFF FOR CALIBRATION
// Online Calibration Function 
float onlineCalibration() {
  digitalWrite(13, HIGH); // Turn on LED when in calibration mode
  delay(1000);
  Serial.println("HELLO! YOU HAVE SUCCESFULLY INTERRUPTED THE PROGRAM");
  delay(1000);

  while (enableInt) {
    while (not leftArrayFull) {
      float average = calculateAverage(leftPosADC, numSamples);
      Serial.println(average);
      if (average > 0) {
        leftArrayFull = true;
        break;
      }
      while (digitalRead(leftPos) == 1) { // Left Position Calibration
        // Collect data
        readADC();
        leftPosADC[index] = adc_reading;
        index++;
        Serial.println("MEASURIIINNNGNGG COLLECTING");
        Serial.println(adc_reading);
        if (index >= numSamples) {
          index = 0;
          break;
        }
        delay(100);
      }
      
     
      delay(100);
      // readADC();
      // Serial.println(adc_reading);
    } 

  enableInt = false;   
  } 

  Serial.println("YOU MAY NOW RESUME YOUR PROGRAM");
  delay(1000);
  digitalWrite(13, LOW);
  

  //////// ONLINE CALIBRATION ////////

//   while (leftPos == 1) { // Left Position Calibration
//     // Collect data
//     leftPosADC[index] = data;
//     index++;
//     if (index >= numSamples) {
//       index = 0;
//       leftArrayFull = true;
//     }

//     if (leftArrayFull) {
//       digitalWrite(12, HIGH); // Turn on led
//       float average = calculateAverage(leftPosADC, numSamples);
//       f2 = average;
//     }

//     delay(100);

//   }

//   while (middlePos == 1); { // Middle Position Calibration
//     // Collect data
//     middlePosADC[index] = data;
//     index++;
//     if (index >= numSamples) {
//       index = 0;
//       middleArrayFull = true;
//     }

//     if (middleArrayFull) {
//       float average = calculateAverage(leftPosADC, numSamples);
//       f1 = average;
//     }

//     delay(100);    

//   }

//   while (rightPos == 1); { // Middle Position Calibration
//     // Collect data
//     rightPosADC[index] = data;
//     index++;
//     if (index >= numSamples) {
//       index = 0;
//       rightArrayFull = true;
//     }

//     if (rightArrayFull) {
//       float average = calculateAverage(leftPosADC, numSamples);
//       f1 = average;
//     }

//     delay(100);    

//   }  
  
//   // Left Position = Pin D3
//   // Middle Position = Pin D4
//   // Right Position = Pin D5
}



////////////////// STUFF THAT WORKS BUT NOT REALLY ?///////////////////////////
void onlineCalibration(float& leftAvg, float& middleAvg, float& rightAvg) {

  digitalWrite(13, HIGH); // Turn on LED when in calibration mode
  delay(1000);
  Serial.println("HELLO! YOU HAVE SUCCESFULLY INTERRUPTED THE PROGRAM");
  delay(1000);

  while (enableInt) {

    Serial.println("YOU HAVE ENTERED THE BIG WAITING ROOM");
    
    if (not leftArrayFull) {
        while (digitalRead(leftPos) == 1) { // Left Position Calibration
        // Collect data
        readADC();
        leftPosADC[index] = adc_reading;
        index++;
        Serial.println("MEASURIIINNNGNGG COLLECTING LEFT");
        Serial.println(adc_reading);
        if (index >= numSamples) {
          index = 0;
          break;
        }
        delay(100);
      }
      float leftAvg = calculateAverage(leftPosADC, numSamples);
      Serial.println(leftAvg);
      if (leftAvg > 0) {
      leftArrayFull = true; 
      }
    }

    else if (not middleArrayFull) {
        while (digitalRead(middlePos) == 1) { // Middle Position Calibration
        // Collect data
        readADC();
        middlePosADC[index] = adc_reading;
        index++;
        Serial.println("MEASURIIINNNGNGG COLLECTING MIDDLE");
        Serial.println(adc_reading);
        if (index >= numSamples) {
          index = 0;
          break;
        }
        delay(100);
      }
      float middleAvg = calculateAverage(middlePosADC, numSamples);
      Serial.println(middleAvg);
      if (middleAvg > 0) {
      middleArrayFull = true;   
      }    
    }

    else if (not rightArrayFull) {
        while (digitalRead(rightPos) == 1) { // Right Position Calibration
        // Collect data
        readADC();
        rightPosADC[index] = adc_reading;
        index++;
        Serial.println("MEASURIIINNNGNGG COLLECTING RIGHT");
        Serial.println(adc_reading);
        if (index >= numSamples) {
          index = 0;
          break;
        }
        delay(100);
      }
      float rightAvg = calculateAverage(rightPosADC, numSamples);
      Serial.println(rightAvg);
      if (rightAvg > 0) {
      rightArrayFull = true;
      }
    }

    else if (leftArrayFull & middleArrayFull & rightArrayFull) {
        enableInt = false;
    }
  }  

  Serial.println("YOU MAY NOW RESUME YOUR PROGRAM");
  delay(1000);
  digitalWrite(13, LOW);
  
}