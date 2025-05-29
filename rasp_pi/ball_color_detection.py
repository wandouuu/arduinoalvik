# Import the necessary packages
import cv2
import numpy as np


cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

KERNEL = np.ones((5, 5), np.uint8)  # This is used for morphology

# You must insert the lower and upper bounds for the colours you wish to detect. Note lighting conditions greatly affect the success rate of this.
ORANGE_LOWER = np.array([6, 120, 160])
ORANGE_UPPER = np.array([26, 255, 255])


while True:

    # Reads from camera
    success, frame = cap.read()

    # Program terminated if the camera cannot be detected
    if not success:
        print("Camera read failed — exiting.")
        break

    # Establishes the HSV colour spectrum
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Mask of object
    orange_mask = cv2.inRange(hsv, ORANGE_LOWER, ORANGE_UPPER)
    mask = orange_mask
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  KERNEL, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, KERNEL, iterations=1)

    # Finds contours of the object with the specified colours
    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # If the contours exist, then
    if contours:
        c = max(contours, key=cv2.contourArea)

        # Only large areas of the colour will be enclosed (eliminates the risk of detecting smaller specs)
        if cv2.contourArea(c) > 300:

            # Identifies the position and center of the object (ping pong ball)
            (x, y), radius = cv2.minEnclosingCircle(c)
            centre = (int(x), int(y))
            radius = int(radius)

            # Circle is drawn around the object of the ping pong ball
            cv2.circle(frame, centre, radius, (0, 255,   0), 2)
            cv2.circle(frame, centre,   3,    (0,   0, 255), -1)
            cv2.putText(frame, "orange ball",
                        (centre[0] - 40, centre[1] - radius - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    cv2.imshow("Mask (debug)", mask)  # OpenCV shows the object
    cv2.imshow("Ball contour", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):  # Quitting the program
        break

cap.release()
cv2.destroyAllWindows()
print("Finished")
