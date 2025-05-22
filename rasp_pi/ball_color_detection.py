import cv2
import numpy as np

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Semih's bounds
# ORANGE_LOWER = np.array([ 6, 120, 160])
# ORANGE_UPPER = np.array([26, 255, 255])

# Brendan's Bounds
ORANGE_LOWER = np.array([4,140,58])
ORANGE_UPPER = np.array([12,214,129])

# WHITE_LOWER  = np.array([  0,   0, 210])
# WHITE_UPPER  = np.array([180,  40, 255])

kernel = np.ones((5, 5), np.uint8)   # this is used for morphology

print("Running")

while True:
    success, frame = cap.read()
    if not success:
        print("Camera read problem")
        break

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    orange_mask = cv2.inRange(hsv, ORANGE_LOWER, ORANGE_UPPER)

    # white_mask  = cv2.inRange(hsv, WHITE_LOWER, WHITE_UPPER)
    # mask = cv2.bitwise_or(orange_mask, white_mask)
    mask = orange_mask

    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        c = max(contours, key=cv2.contourArea)
        if cv2.contourArea(c) > 300:        #will make it skip tiny specs
            (x, y), radius = cv2.minEnclosingCircle(c)
            centre = (int(x), int(y)); radius = int(radius)

            cv2.circle(frame, centre, radius, (0, 255,   0), 2)
            cv2.circle(frame, centre,   3,    (0,   0, 255), -1)
            cv2.putText(frame, "orange ball",
                        (centre[0] - 40, centre[1] - radius - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    cv2.imshow("Mask (debug)", mask)
    cv2.imshow("Ball contour", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
print("Finished")
