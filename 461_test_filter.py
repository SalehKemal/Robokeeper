import cv2
import numpy as np
import math
import serial
import matplotlib.pyplot as plt
import time
# Open serial connection to Arduino
ser = serial.Serial('COM4', 200000, timeout=0.1)
time.sleep(1)  # Allow time for Arduino to reset
ser.flushInput()  # Clear input buffer
#resolution to shoot @260 fps
width = 640
height = 360

# Initialize last_sent variable
last_sent = 0
center_x = width / 2         # calculate center of frame(Xc,Yc)
center_y = height / 2

# initate video capture with resolution and fps. fps set to 300 to achieve max
cap = cv2.VideoCapture(0)  # change number to which ever camera u want to use.
cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
cap.set(cv2.CAP_PROP_FPS, 300)

# function to rotate servo. You firrst need to upload servo_control to uno
#def set_servo_angle(angle):
   # ser.write((str(angle) + '\n').encode())
target_gear=0

def set_target(angle, last_sent, target_gear):
    rolling_window = 5

    if abs(angle) < 90:
        target = 0 
    elif abs(angle) >= 90:
        target = 180
    
    target_gear=target*(800/360)
    # Initialize a list to store previous targets
    if 'previous_targets' not in set_target.__dict__:
        set_target.previous_targets = []

    # Add current target to the list
    set_target.previous_targets.append(target_gear)

    # Trim list to rolling window size
    if len(set_target.previous_targets) > rolling_window:
        set_target.previous_targets = set_target.previous_targets[-rolling_window:]

    # Calculate average of previous targets
    rolling_average = sum(set_target.previous_targets) / len(set_target.previous_targets)

    # Update target gear based on rolling average
    target_gear = rolling_average

    # Write to serial
    ser.write(f"{int(target_gear)}\n".encode())

    return last_sent, target_gear





#def linear_map(value, from_min, from_max, to_min, to_max):
    # Calculate the percentage of value within the from range
    #ratio = (value - from_min) / (from_max - from_min)
    
    # Map the percentage to the to range
    #mapped_value = ratio * (to_max - to_min) + to_min
    #return mapped_value

# function to calculate angle w/repsct to center of frame. this will work if servo is positioned right under camera center. 
# else we would need to first find servo from frame and then do the angle with respect to servo
def calculate_angle(x, y, center_x, center_y):

    #perform offset to have (0,0) to be Xc and Yc. Then perfform angle calc. Change rad to degree
    angle_rad = math.atan2(center_y-y, x - center_x)  # center y -y to ensure above 
    angle_deg = math.degrees(angle_rad)
    return angle_deg

# calculate center using moments
def center_moments(max_contour,radius):
    M = cv2.moments(max_contour)  
    if radius > 10:   # radius threshold to elimante noise
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
    else:
         center=(-1,1)
    return center

# Define color range of the ball in HSV. This lower and upper arrays works for the green tennis balls. Note: lighting affects HSV
# use coloradjust.py to find appropriate HSV range
lower_yellow = np.array([15, 60, 165])
upper_yellow = np.array([30, 255, 255])
m=0
data = {'target': [], 'pos': []}
prev_pos=0
for m in range(2000):
    ts=time.time()
    ret, frame = cap.read()            # read frame, ret is a boolean. if false, frame is not read
    frame = cv2.flip(frame, 1)         # flip image so its nor mirrored. 

    cv2.circle(frame, (int(center_x), int(center_y)), 5, (0, 255, 0), -1)  # draw a circle at center of frame 

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) # change frame from BGR space to HSV to classify by color 

    # Threshold the HSV to only get yellow
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    
    #additional masks for high freq noises
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # only proceed if at least one contour was found
    if contours:
        # find the largest contour in the mask, then compute circle and center
        c = max(contours, key=cv2.contourArea)

        ((x, y), radius) = cv2.minEnclosingCircle(c)   #(x,y) is the center of the cotnour. 
        cv2.circle(frame, (int(x), int(y)) , 5, (0, 0, 255), -1)  # Draw a small green circle at the center

        if radius>10:   # radius threshold to elimante any other noise detected
            x_adjusted = x - center_x
            y_adjusted = y - center_y
        else:
            (x_adjusted, y_adjusted)=(-1,-1)  # if radius is too small just set to -1,-1

        #print coordinates on screen
        coordinates_text = "Coordinates: ({:.2f}, {:.2f})".format(x_adjusted, y_adjusted) 
        cv2.putText(frame, coordinates_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        center_m=center_moments(c,radius)
        #cv2.circle(frame, center, 5, (0, 0, 255), -1)   # draw the center calculate by moments. 
        center_madjust = (center_m[0] - center_x, center_m[1] - center_y)   # adjust coordinates calculated by moments

        #print moemnts coordinates on screen
        coordinates_mtext = "Coordinates_using moments: ({:.2f}, {:.2f})".format(center_madjust[0], center_madjust[1]) 
        cv2.putText(frame, coordinates_mtext, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # Draw a blue line from center of frame to ball
        cv2.line(frame, (int(center_x), int(center_y)), (int(x), int(y)), (255, 0, 0), 2)

        # Calculate angle with respect to the center
        angle = calculate_angle(x, y, center_x, center_y)
    
        # angle using moments
        #angle=calculate_angle(center_m[0], center_m[1], center_x, center_y)    

        #print angle on screen
        angle_text = "Angle: {:.2f} degrees".format(angle)
        cv2.putText(frame, angle_text, (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # set servo to absolute value of angle. since sevo only does o to 180
        #set_servo_angle(abs(angle))
        [last_sent,target_gear]=set_target(angle,last_sent,target_gear)

            # Read position data
    pos = ser.readline().decode().strip()
    print(f"Position: {pos}")
    #te=time.time()
    #print(te-ts)
        
        # Check if pos is an empty string
    if pos:
        prev_pos=pos
        data['target'].append(target_gear)
        data['pos'].append(int(pos))
        #print(1)
    else:
        # print(1)
         print("Empty position signal received")
         data['target'].append(target_gear)
         data['pos'].append(int(prev_pos))
    #time.sleep(1)
    # show final image
    cv2.imshow('Frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):  # stop showing image when q is pressed
            break
    m=m+1
# Release the video capture and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()

# Plotting
plt.plot(data['target'], label='Target')
plt.plot(data['pos'], label='Position')
plt.xlabel('Time')
plt.ylabel('Value')
plt.title('Target and Position Signals')
plt.legend(loc='upper right')
plt.show()