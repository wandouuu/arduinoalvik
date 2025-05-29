import serial, time
from picamera import PiCamera
import time

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
time.sleep(2)

camera = PiCamera()

# Loops until it receives a photo string from the Alvik
while True:
	# Reads serial data from Alvik
	line = ser.readline().decode('utf-8').strip()
	if line == "photo":
		time.sleep(2)
		# Captures photo from Pi Camera and stores it as a file in a certain directory
		camera.capture("/directory1/directory2/nameofphoto.jpg")
		# Writes to ALvik that the photo has been taken
		ser.write(b'2')
		break
		
