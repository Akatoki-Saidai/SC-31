import RPi.GPIO as GPIO
class GPIOsetup:
    def __init__(self):
        ## GPIO setup
        GPIO.setmode(GPIO.BCM)
        ## defined GPIO pins for the following sc31cirkitv1 -> https://drive.google.com/drive/folders/1mVAq-eKdAjtUOeKXxuRbDCvJjKMWEASI?usp=drive_link
        self.MDctrl = 9
        self.GNSSreset = 19

        self.Amin1 = 17
        self.Amin2 = 27
        self.Bmin1 = 4
        self.Bmin2 = 18
        self.AmphaseA = 10
        self.AmphaseB = 22
        self.BmphaseA = 24
        self.BmphaseB = 23

        self.Servo1 = 12
        self.Servo2 = 13