# Import the necessary packages
from imutils import paths
import numpy as np
import imutils
import cv2
import os


KERNEL = np.ones((5, 5), np.uint8)

# You must insert the lower and upper bounds for the colours you wish to detect. Note lighting conditions greatly affect the success rate of this.
ORANGE_LOWER = np.array([6, 120, 160])
ORANGE_UPPER = np.array([26, 255, 255])

# Function that identifies and returns the position and radius of the object of the colour you're trying to detect


def find_marker(image):

    # Declares colour spectrum
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Mask of object
    mask = cv2.inRange(hsv, ORANGE_LOWER, ORANGE_UPPER)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, KERNEL, iterations=1)

    # Finds countours of the object with the colour
    cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)

    # If no contour is detected around object
    if not cnts:
        return None
    c = max(cnts, key=cv2.contourArea)

    # Minimizes the risk of detecting dots with that colour (ensures the entire ping pong ball is enclosed, not just specific dots with the specific colour)
    if cv2.contourArea(c) < 200:
        return None

    # Position as well as the radius of the object is returned
    (x, y), radius = cv2.minEnclosingCircle(c)
    return (x, y), radius

# Function that calculates the distance of the object to the camera


def distance_to_camera(knownWidth_cm, focalLength_px, perWidth_px):
    return (knownWidth_cm * focalLength_px) / perWidth_px


# Path where the photo of the calibration photo is taken (You must take a new one every time with snap_photo.py if you go into a new environment)
calib_path = "/directory1/directory2/calibration_photo.jpg"

# To calibrate the calculations, you must measure beforehand the distance of the object from the camera and its width
KNOWN_DISTANCE_CM = 33.50
KNOWN_WIDTH_CM = 4.0

# If no calibration photo is found, a FileNotFoundError is raised
if not os.path.isfile(calib_path):
    raise FileNotFoundError(f"calibration image missing")

# The calibration image is read provided there is a path towards an existing one
calib_image = cv2.imread(calib_path)

# Finds the position as well as the radius of the object
res = find_marker(calib_image)

# If no marker is found, raises an error
if res is None:
    raise RuntimeError("No ball found")

# Variables for the camera
(center0, radius0) = res
perWidth0 = 2 * radius0
focalLength_px = (perWidth0 * KNOWN_DISTANCE_CM) / KNOWN_WIDTH_CM

# Creates window with the image for the photos below
cv2.namedWindow("image", cv2.WINDOW_NORMAL)
cv2.resizeWindow("image", 800, 600)

# Checks the photos and identifies the ball in the image (if there is one)
for imagePath in sorted(paths.list_images("/home/waterlooletmein/images")):

    # Reads the image and determines if the colour is found by marking the object
    image = cv2.imread(imagePath)
    res = find_marker(image)

    # If no ball is detected, then the distance is set to the NoneType
    if res is None:
        dist_cm = None

    # Otherwise, a ball is detected and it calculates the distance in front of it
    else:
        (x, y), radius = res
        perWidth = 2 * radius
        # Distance calculated
        dist_cm = distance_to_camera(KNOWN_WIDTH_CM, focalLength_px, perWidth)

        # Position of center of object (ping pong ball) is identified
        center = (int(x), int(y))
        # Circle created around the object with the specified colour range
        cv2.circle(image, center, int(radius), (0, 255, 0), 2)
        cv2.circle(image, center, 3, (0, 0, 255), -1)

    # If the object was detected, then text of its distance is displayed on the image of the object
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

    # Image of the object is shown and enclosed unless the colour is not detected
    cv2.imshow("image", image)
    key = cv2.waitKey(0) & 0xFF
    if key == ord('q'):
        break

cv2.destroyAllWindows()
