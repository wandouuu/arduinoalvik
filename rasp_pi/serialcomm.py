# Import the necessary packages
import serial
import time
from picamera import PiCamera
import time
import math
from imutils import paths
import numpy as np
import imutils
import cv2
import os


KERNEL = np.ones((5, 5), np.uint8)

# You must insert the lower and upper bounds for the colours you wish to detect. Note lighting conditions greatly affect the success rate of this.
ORANGE_LOWER = np.array([4, 140, 58])  # HSV Values
ORANGE_UPPER = np.array([12, 214, 129])  # HSV Values

# Pi establishes serial communication with the Alvik
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
time.sleep(2)

# Initialize the camera object for taking picture of the ball
camera = PiCamera()

# Function that identifies and returns the position and radius of the object of the colour you're trying to detect


def find_marker(image):

    # Declares colour spectrum
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Mask of the object
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


# To calibrate the calculations, you must measure beforehand the distance of the object from the camera and its width
KNOWN_DISTANCE_CM = 18.25
KNOWN_WIDTH_CM = 4

# Path where the photo of the calibration photo is taken (You must take a new one every time with snap_photo.py if you go into a new environment)
calib_path = "/directory1/directory2/calibration_photo.jpg"

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

# The Pi continues to read from the USB port for its signal to take a photo
while True:
    # Pi reads the messages in the Serial bus
    line = ser.readline().decode('utf-8').strip()

    # If the message is photo, then it captures a photo of the ball in front of it (if there is one)
    if line == "photo":
        time.sleep(2)

        # Photo is stored in a directory on your PC of your choosing
        camera.capture(
            "/directory1/directory2/folder_with_images/distance.jpg")

        # Checks the photos and identifies the ball in the image (if there is one)
        for imagePath in sorted(paths.list_images("/directory1/directory2/folder_with_images")):

            # Reads the image and determines if the colour is found by marking the object
            image = cv2.imread(imagePath)
            res = find_marker(image)

        # If no ball is detected, the message serially communicated to Alvik is 0 meters (i.e. no ball is in front of it)
        if res is None:
            dist_cm = 0
            print("No ball found.")
            message = f"{str(math.floor(dist_cm))}"
            encoded_message = message.encode()
            ser.write(encoded_message)

        # Otherwise, a ball is detected and it calculates the distance in front of it
        else:
            (x, y), radius = res
            perWidth = 2 * radius
            dist_cm = distance_to_camera(
                KNOWN_WIDTH_CM, focalLength_px, perWidth)
            message = f"{str(math.floor(dist_cm))
                         }"
            encoded_message = message.encode()
            ser.write(encoded_message)
        time.sleep(5)
        break
