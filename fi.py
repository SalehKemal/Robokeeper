import serial
import time
import matplotlib.pyplot as plt

# Open serial connection to Arduino
ser = serial.Serial('COM4', 200000, timeout=0.1)
time.sleep(1)  # Allow time for Arduino to reset
ser.flushInput()  # Clear input buffer

T = 7000
Hg = 400
lo = 0
target = 0
previous_millis = 0
data = {'target': [], 'pos': []}
prev_pos=0
m=0
while(m<(200000)):
    ts=time.time()
    current_millis = int(round(time.time() * 1000))  # Get the current time in milliseconds
    if current_millis - previous_millis >= T/2:
        previous_millis = current_millis  # Save the last time the output was toggled
            
            # Toggle the target value between high and low
        if target == lo:
            target = Hg
        else:
            target = lo
        
    ser.write(f"{int(target)}\n".encode())
    print("target:",target)
   # print(f"Target: {target}")

        # Read position data
    pos = ser.readline().decode().strip()
    print(f"Position: {pos}")
    
    te=time.time()
    #print(te-ts)
        # Check if pos is an empty string
    if pos:
        prev_pos=pos
        data['target'].append(target)
        data['pos'].append(int(pos))
    else:
         print("Empty position signal received")
         data['target'].append(target)
         data['pos'].append(int(prev_pos))
    m=m+1
    print(m)
        
# Plotting
plt.plot(data['target'], label='Target')
plt.plot(data['pos'], label='Position')
plt.xlabel('Time')
plt.ylabel('Value')
plt.title('Target and Position Signals')
plt.legend(loc='upper right')
plt.show()
