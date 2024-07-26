import serial
import time

# Data Points from calibration
# x0 = 995.6491
# x1 = 523.3557
# x2 = 26.8174
# y0 = 30
# y1 = 0
# y2 = -30

# # Open serial connection to Arduino 
# arduino = serial.Serial('COM5', 9600, timeout=1)  
# # Wait for the connection to establish
# time.sleep(2)  

# # Read the serial monitor for data
# while True:
#     if arduino.in_waiting > 0:
#         x = arduino.readline().decode('utf-8').strip()
#         if x.isdigit():
#             x = int(x)
#             # Interpolation Calculation
#             interpolate1 = (((x-x1)*(x-x2))/((x0-x1)*(x0-x2))) * y0
#             interpolate2 = (((x-x0)*(x-x2))/((x1-x0)*(x1-x2))) * y1
#             interpolate3 = (((x-x0)*(x-x1))/((x2-x0)*(x2-x1))) * y2
#             interpolateVal = interpolate1 + interpolate2 + interpolate3
#             # Write back to serial monitor
#             arduino.write(f"{interpolateVal}\n".encode('utf-8'))
#             print(f"ADC Value: {x}, Angle: {interpolateVal}")


# TIMER INFORMATION FOR ARDUINO SCRIPT
#   // // FAST PWM 
#   // // Clear Timer1 control registers
#   // TCCR1A = 0;
#   // TCCR1B = 0;

#   // // Set Timer1 to Fast PWM mode with ICR1 as top value
#   // TCCR1A |= (1 << WGM11);
#   // TCCR1B |= (1 << WGM12) | (1 << WGM13);

#   // // Set non-inverting mode for PWM on OC1A (Pin 9)
#   // TCCR1A |= (1 << COM1A1);

#   // // Set the prescaler to 8 and start the timer
#   // TCCR1B |= (1 << CS11);

#   // // Set the PWM frequency by setting the ICR1 value
#   // ICR1 = 199;  // This sets the frequency to 10kHz

#   // // Set the initial duty cycle
#   // OCR1A = 0; 
            
#   //////// ARDUINO & PYTHON COMMUNICATION ////////

#   // // Check if the serial monitor contains any data from python
#   // if (Serial.available() > 0) { 
#   //   // Parse information from python
#   //   int interpolateVal = Serial.parseFloat(); 
#   //   // Output information back to the serial monitor 
#   //   Serial.print("Modified Value: ");
#   //   Serial.println(interpolateVal);
#   // }


