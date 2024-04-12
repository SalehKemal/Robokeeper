import serial
import time

#serial connection to Arduino
ser = serial.Serial('/dev/cu.usbserial-1420', 9600)  

def set_servo_angle(angle):
    ser.write((str(angle) + '\n').encode())

#test
try:
    while True:
        # Move servo from 0 to 180 degrees
        for angle in range(0, 180):
            set_servo_angle(240)
            time.sleep(0.02)  # Adjust delay as needed
        # Move servo from 180 to 0 degrees
        for angle in range(180, -1, -1):
            set_servo_angle(0)
            time.sleep(0.02)  # Adjust delay as needed

except KeyboardInterrupt:
    ser.close()  # Close serial connection on Ctrl+C
