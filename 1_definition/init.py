import RPi.GPIO as GPIO

## GPIO setup
GPIO.setmode(GPIO.BCM)
## defined GPIO pins for the following sc31cirkitv1 -> https://drive.google.com/drive/folders/1mVAq-eKdAjtUOeKXxuRbDCvJjKMWEASI?usp=drive_link
MDctrl = 9
GNSSreset = 19

Amin1 = 17
Amin2 = 27
Bmin1 = 4
Bmin2 = 18
AmphaseA = 10
AmphaseB = 22
BmphaseA = 24
BmphaseB = 23

Servo1 = 12
Servo2 = 13