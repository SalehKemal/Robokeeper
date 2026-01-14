import cv2
import numpy as np

#handle trackbar changes
def on_trackbar(val):
    pass

# Initialize video capture
cap = cv2.VideoCapture(0)

# Create a window
cv2.namedWindow("Color Filter")

# Define initial values for trackbars
initial_low = np.array([30, 65, 100], dtype=np.uint8)
initial_high = np.array([60, 255, 255], dtype=np.uint8)

# trackbars
cv2.createTrackbar("Low H", "Color Filter", initial_low[0], 179, on_trackbar)
cv2.createTrackbar("High H", "Color Filter", initial_high[0], 179, on_trackbar)
cv2.createTrackbar("Low S", "Color Filter", initial_low[1], 255, on_trackbar)
cv2.createTrackbar("High S", "Color Filter", initial_high[1], 255, on_trackbar)
cv2.createTrackbar("Low V", "Color Filter", initial_low[2], 255, on_trackbar)
cv2.createTrackbar("High V", "Color Filter", initial_high[2], 255, on_trackbar)

while True:
    # Read frame from camera
    ret, frame = cap.read()
    if not ret:
        break

    # Convert frame to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Get current trackbar values
    low_h = cv2.getTrackbarPos("Low H", "Color Filter")
    high_h = cv2.getTrackbarPos("High H", "Color Filter")
    low_s = cv2.getTrackbarPos("Low S", "Color Filter")
    high_s = cv2.getTrackbarPos("High S", "Color Filter")
    low_v = cv2.getTrackbarPos("Low V", "Color Filter")
    high_v = cv2.getTrackbarPos("High V", "Color Filter")

    # Create lower and upper bounds for the color filter
    lower_bound = np.array([low_h, low_s, low_v])
    upper_bound = np.array([high_h, high_s, high_v])

    # Apply color filter
    mask = cv2.inRange(hsv, lower_bound, upper_bound)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    result = cv2.bitwise_and(frame, frame, mask=mask)

    # Display the original frame and the result
    cv2.imshow("Original", frame)
    cv2.imshow("Color Filter Result", result)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()
