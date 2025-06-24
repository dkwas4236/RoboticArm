import cv2
import numpy as np

def nothing(x):
    pass

# Create window with trackbars for HSV ranges
cv2.namedWindow("HSV Calibration")
cv2.createTrackbar("H Min", "HSV Calibration", 0, 179, nothing)
cv2.createTrackbar("H Max", "HSV Calibration", 179, 179, nothing)
cv2.createTrackbar("S Min", "HSV Calibration", 0, 255, nothing)
cv2.createTrackbar("S Max", "HSV Calibration", 255, 255, nothing)
cv2.createTrackbar("V Min", "HSV Calibration", 0, 255, nothing)
cv2.createTrackbar("V Max", "HSV Calibration", 255, 255, nothing)

cap = cv2.VideoCapture(2)  # Pi Cam or USB cam

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame = cv2.resize(frame, (640, 480))
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Get current trackbar positions
    h_min = cv2.getTrackbarPos("H Min", "HSV Calibration")
    h_max = cv2.getTrackbarPos("H Max", "HSV Calibration")
    s_min = cv2.getTrackbarPos("S Min", "HSV Calibration")
    s_max = cv2.getTrackbarPos("S Max", "HSV Calibration")
    v_min = cv2.getTrackbarPos("V Min", "HSV Calibration")
    v_max = cv2.getTrackbarPos("V Max", "HSV Calibration")

    # Create HSV mask
    lower = np.array([h_min, s_min, v_min])
    upper = np.array([h_max, s_max, v_max])
    mask = cv2.inRange(hsv, lower, upper)
    result = cv2.bitwise_and(frame, frame, mask=mask)

    cv2.imshow("Original", frame)
    cv2.imshow("Mask", mask)
    cv2.imshow("Filtered Result", result)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
