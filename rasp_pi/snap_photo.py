import serial, time
from picamera import PiCamera
import time

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
time.sleep(2)

camera = PiCamera()



while True:
	line = ser.readline().decode('utf-8').strip()
	if line == "photo":
		time.sleep(2)
		camera.capture("/home/waterlooletmein/images/bren_photo1.jpg")
		ser.write(b'2')
		break
		
