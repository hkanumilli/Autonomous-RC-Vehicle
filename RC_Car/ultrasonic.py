import time
import RPi.GPIO as GPIO

from socket import *

# Squash warnings
GPIO.setwarnings(False)

# GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BCM)

# GPIO Pins
GPIO_TRIGGER = 18
GPIO_ECHO = 24

# Set GPIO direciton (IN/OUT)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)

def measure_distance():
    # Set the trigger to high
    GPIO.output(GPIO_TRIGGER, True)
    
    # Set the trigger to low after some time
    time.sleep(0.0000001)
    GPIO.output(GPIO_TRIGGER, False)
    
    # StartTime, StopTime
    startTime = time.time()
    stopTime = time.time()
    
    # Save StartTime
    while GPIO.input(GPIO_ECHO) == 0:
        startTime = time.time()
    
    while GPIO.input(GPIO_ECHO) == 1:
        stopTime = time.time()
        
    # return distance
    return ((stopTime - startTime) * 34300) / 2


# if __name__ == '__main__':
#     try:
#         while True:
#             distance = measure_distance()
#             if distance < 10:
#                 print('Obstacle in front - {} cm, stopping vehicle!'.format(distance))
#             time.sleep(1)
#     except Exception as e:
#         print('Stopped accessing ultrasonic sensor, due to following error: {}'.format(e))
#         GPIO.cleanup()
