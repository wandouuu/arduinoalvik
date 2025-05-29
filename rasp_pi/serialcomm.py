import serial, time
from picamera import PiCamera
import time
import math

# import the necessary packages
from imutils import paths
import numpy as np
import imutils
import cv2
import os

# Semih's Bounds
# ORANGE_LOWER = np.array([6, 120, 160])    
# ORANGE_UPPER = np.array([26, 255, 255])   
KERNEL = np.ones((5, 5), np.uint8)       
# Brendan's Bounds
ORANGE_LOWER = np.array([4,140,58])
ORANGE_UPPER = np.array([12,214,129])

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
time.sleep(2)

camera = PiCamera()

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

# Semih's Known Stuff
# KNOWN_DISTANCE_CM = 33.50    
# KNOWN_WIDTH_CM    = 4.0

# Brendan's Known Stuff
KNOWN_DISTANCE_CM = 18.25
KNOWN_WIDTH_CM = 4

calib_path = "/home/waterlooletmein/calib_bren.jpg"
if not os.path.isfile(calib_path):
    raise FileNotFoundError(f"calibration image missing")

calib_image = cv2.imread(calib_path)
res = find_marker(calib_image)
if res is None:
    raise RuntimeError("No ball found")
(center0, radius0) = res
perWidth0 = 2 * radius0 
focalLength_px = (perWidth0 * KNOWN_DISTANCE_CM) / KNOWN_WIDTH_CM


while True:
	line = ser.readline().decode('utf-8').strip()
	if line == "photo":
		time.sleep(2)
		camera.capture("/home/waterlooletmein/images/dist.jpg")
		for imagePath in sorted(paths.list_images("/home/waterlooletmein/images")):
			image = cv2.imread(imagePath)
			res = find_marker(image)
		if res is None:
			dist_cm = 0
        	print("No ball found.")
            message = f"{str(math.floor(dist_cm))}"
			encoded_message = message.encode()
			ser.write(encoded_message)
		else:
			(x, y), radius = res
			perWidth = 2 * radius
			dist_cm = distance_to_camera(KNOWN_WIDTH_CM, focalLength_px, perWidth)
			message = f"{str(math.floor(dist_cm))
			}"
			encoded_message = message.encode()
			ser.write(encoded_message)
		time.sleep(5)
		break
		
