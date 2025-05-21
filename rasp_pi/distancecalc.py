# import the necessary packages
from imutils import paths
import numpy as np
import imutils
import cv2
import os

ORANGE_LOWER = np.array([6, 120, 160])    
ORANGE_UPPER = np.array([26, 255, 255])   
KERNEL = np.ones((5, 5), np.uint8)       


def find_marker(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, ORANGE_LOWER, ORANGE_UPPER)

    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, KERNEL, iterations=1)

    cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    if not cnts:
        return None
    c = max(cnts, key=cv2.contourArea)
    if cv2.contourArea(c) < 200:
        return None
    (x, y), radius = cv2.minEnclosingCircle(c)
    return (x, y), radius

def distance_to_camera(knownWidth_cm, focalLength_px, perWidth_px):
    return (knownWidth_cm * focalLength_px) / perWidth_px


KNOWN_DISTANCE_CM = 33.50    
KNOWN_WIDTH_CM    = 4.0    

calib_path = "/home/waterlooletmein/calibcalc.jpg"
if not os.path.isfile(calib_path):
    raise FileNotFoundError(f"calibration image missing")

calib_image = cv2.imread(calib_path)
res = find_marker(calib_image)
if res is None:
    raise RuntimeError("No ball found")
(center0, radius0) = res
perWidth0 = 2 * radius0 
focalLength_px = (perWidth0 * KNOWN_DISTANCE_CM) / KNOWN_WIDTH_CM


cv2.namedWindow("image", cv2.WINDOW_NORMAL)
cv2.resizeWindow("image", 800, 600)

for imagePath in sorted(paths.list_images("/home/waterlooletmein/images")):
    image = cv2.imread(imagePath)
    res = find_marker(image)
    if res is None:
        dist_cm = None
    else:
        (x, y), radius = res
        perWidth = 2 * radius
        dist_cm = distance_to_camera(KNOWN_WIDTH_CM, focalLength_px, perWidth)

        center = (int(x), int(y))
        cv2.circle(image, center, int(radius), (0, 255, 0), 2)
        cv2.circle(image, center, 3, (0, 0, 255), -1)

    if dist_cm is not None:
        cv2.putText(
            image,
            f"{dist_cm:.1f} cm",
            (10, image.shape[0] - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 255, 0),
            2
        )

    cv2.imshow("image", image)
    key = cv2.waitKey(0) & 0xFF
    if key == ord('q'):
        break

cv2.destroyAllWindows()
