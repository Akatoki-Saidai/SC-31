import init_GPIO
GPIO = init_GPIO.GPIOsetup()
import init_GPIO
import time
import cv2
from picamera import PiCamera
from picamera.array import PiRGBArray

GPIO = init_GPIO.GPIOsetup()


# Camera
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 30

raw_capture = PiRGBArray(camera, size=(640, 480))

time.sleep(0.1)


def cameraRead():
    camera.capture(raw_capture, format="bgr", use_video_port=True)

    frame = raw_capture.array.copy()

    raw_capture.truncate(0)

    return frame