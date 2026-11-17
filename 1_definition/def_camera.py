import time
import cv2
from picamera import PiCamera
from picamera.array import PiRGBArray

class camera:
    def __init__():
        # Camera
        width_cons = 640
        height_cons = 480
        camera = PiCamera()
        camera.resolution = (640, 480)
        camera.framerate = 30
        raw_capture = PiRGBArray(camera, size=(640, 480))


    def cameraRead():
        camera.capture(raw_capture, format="bgr", use_video_port=True)

        frame = raw_capture.array.copy()

        raw_capture.truncate(0)

        return frame