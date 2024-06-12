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
            



