import cv2
import numpy as np
import math
import serial
import matplotlib.pyplot as plt
import time
import threading

# Open serial connection to Arduino
ser = serial.Serial('COM6', 19200, timeout=0.1)
time.sleep(1)  # Allow time for Arduino to reset
ser.flushInput()  # Clear input buffer

#resolution to shoot @260 fps
width = 640
height = 360

# Initialize last_sent variable
last_sent = 0
center_x = width / 2         # calculate center of frame(Xc,Yc)
center_y = height / 2

# Initialize data dictionary globally
data = {'target': [], 'pos': []}
prev_pos = 0

# initate video capture with resolution and fps. fps set to 300 to achieve max
cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)   # change number to which ever camera u want to use.
cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
cap.set(cv2.CAP_PROP_FPS, 300)

# Define color range of the ball in HSV. This lower and upper arrays works for the green tennis balls. Note: lighting affects HSV
# use coloradjust.py to find appropriate HSV range
lower_yellow = np.array([15, 60, 165])
upper_yellow = np.array([30, 255, 255])

# Function to rotate servo
def set_target(angle, last_sent):
    if angle == -1111:  # Termination signal
        last_sent = -1
        ser.write(f"{int(last_sent)}\n".encode())
        print("sent")
        return last_sent
    else:
        if abs(angle) < 90:
            target = -90 
        elif abs(angle) >= 90:
            target = 90

        if abs(last_sent - target) > 70:
            last_sent = target
            return last_sent
        else:

            return last_sent

# Function to calculate angle w/repsct to center of frame
def calculate_angle(x, y, center_x, center_y):
    angle_rad = math.atan2(center_y - y, x - center_x)
    angle_deg = math.degrees(angle_rad)
    return angle_deg

# Function to continuously capture frames and process them
def camera_thread(duration):
    global last_sent

    start_time = time.time()
    while time.time() - start_time < duration:
        ret, frame = cap.read()
        frame = cv2.flip(frame, 1)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            c = max(contours, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)

            if radius > 10:
                x_adjusted = x - center_x
                y_adjusted = y - center_y
            else:
                (x_adjusted, y_adjusted) = (-1, -1)

            angle = calculate_angle(x, y, center_x, center_y)
            last_sent = set_target(angle, last_sent)
        cv2.imshow('Frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):  # stop showing image when q is pressed
            break
    set_target(-1111, last_sent)    


# Function to continuously read Arduino position and update data for plotting
def arduino_thread(duration):
    global prev_pos

    start_time = time.time()
    while time.time() - start_time < duration:
        target_gear=last_sent * (800 / 360)
        ser.write(f"{int(target_gear)}\n".encode())
        print(target_gear)

        pos = ser.readline().decode().strip()
        print(f"Position: {pos}")
        
        if pos:
            prev_pos = pos
            data['target'].append(target_gear)
            data['pos'].append(pos)
        else:
            pos=prev_pos
            data['target'].append(target_gear)
            data['pos'].append(pos)



# Define duration for the threads to run (in seconds)
duration = 80
# Start camera and Arduino threadsq
camera_thread = threading.Thread(target=camera_thread, args=(duration,))
arduino_thread = threading.Thread(target=arduino_thread, args=(duration,))

camera_thread.start()
arduino_thread.start()

# Wait for threads to finish
camera_thread.join()
arduino_thread.join()

# Release the video capture and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()

data['pos'] = [int(pos) for pos in data['pos']]

# Plotting
plt.plot(data['target'], label='Target')
plt.plot(data['pos'], label='Position')
plt.xlabel('Time')
plt.ylabel('Value')
plt.title('Target and Position Signals')
plt.legend(loc='upper right')
plt.show()
