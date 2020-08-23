import time
import requests
import RPi.GPIO as GPIO
import adafruit_mlx90614
import board
import busio as io
import cv2
import pyzbar.pyzbar as pyzbar
import pygame

pygame.init()

beep = pygame.mixer.Sound('beep.wav')
clear = pygame.mixer.Sound('clear.wav')
alert = pygame.mixer.Sound('alert.wav')

i2c = io.I2C(board.SCL, board.SDA, frequency=100000)
mlx = adafruit_mlx90614.MLX90614(i2c)

URL = ' /check-inspection'#server req

pin = 18 #servo

trig = 13 # ultrasonic
echo = 19

GPIO.setmode(GPIO.BCM)
GPIO.setup(pin,GPIO.OUT)

GPIO.setup(trig, GPIO.OUT)
GPIO.setup(echo, GPIO.IN)

p = GPIO.PWM(pin, 50)
p.start(0)

cap = cv2.VideoCapture(0)

#camera resolution
cap.set(3,640)
cap.set(4,480)

def distance():
    GPIO.output(trig, False)
    time.sleep(0.5)
    GPIO.output(trig, True)
    time.sleep(0.00001)
    GPIO.output(trig, False)

    while GPIO.input(echo) == 0:
        pulse_start = time.time()

    while GPIO.input(echo) == 1:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    d = pulse_duration * 17000
    d = round(distance, 2)
    return d

def check_id(qr_data):
    data = {'qstnCrtfcNoEncpt': qr_data, 'temp': ''}
    res = requests.post(URL, data=data)
    if res.status_code == 200:
        return True
    else:
        return False

def send_temp(qr_data, temp):
#    temp = mlx.object_temperature
    data = {'qstnCrtfcNoEncpt' : qr_data, 'temp' : temp}
    requests.post(URL, data=data)
    #return temp





#-------------set----------------
init_distance = distance()

while(cap.isOpened()):
    ret, img = cap.read()

    if not ret:
        continue

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    decoded = pyzbar.decode(gray)



    for d in decoded:

        x, y, w, h = d.rect

        barcode_data = d.data.decode("utf-8")
        barcode_type = d.type

        text = '%s (%s)' % (barcode_data, barcode_type)

        beep.play()

        if check_id(text):
            while(True):
                if (init_distance - distance()) > 3: # hand detect
                    t = bool(mlx.object_temperature < 33)
                    if t:
                        clear.play()
                    else:
                        alert.play()
                    send_temp(text, t)

                    p.ChangeDutyCycle(1)
                    # time.sleep(0.05)
                    p.ChangeDutyCycle(5)
                    break

    cv2.imshow('img', img)
    key = cv2.waitKey(1)
    if key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
p.stop()
GPIO.cleanup()









