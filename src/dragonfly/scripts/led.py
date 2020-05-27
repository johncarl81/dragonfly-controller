#! /usr/bin/env python
import argparse
import time
import RPi.GPIO as GPIO

# GPIO setup.
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

class LED:
    def __init__(self, red=18, green=17, blue=4):
        GPIO.setup(red, GPIO.OUT)
        GPIO.setup(green, GPIO.OUT)
        GPIO.setup(blue, GPIO.OUT)

        # Set up colors using PWM so we can control individual brightness.
        self.red = GPIO.PWM(red, 100)
        self.green = GPIO.PWM(green, 100)
        self.blue = GPIO.PWM(blue, 100)
        self.red.start(100)
        self.green.start(100)
        self.blue.start(100)

    def __enter__(self):
        return self

    def __exit__(self):
        shutdown(self)

    def shutdown(self):
        self.setColor([0, 0, 0])
        self.red.stop()
        self.green.stop()
        self.blue.stop()

       
    # Set a color by giving R, G, and B values of 0-255.
    def setColor(self, rgb = []):
        # Convert 0-255 range to 0-100.
        rgb = [(x / 255.0) * 100 for x in rgb]
        self.red.ChangeDutyCycle(100 - rgb[0])
        self.green.ChangeDutyCycle(100 - rgb[1])
        self.blue.ChangeDutyCycle(100 - rgb[2])

if __name__ == '__main__':
    # Get RGB colors from command line arguments.
    parser = argparse.ArgumentParser(description = 'Light up the world!')
    parser.add_argument('rgb', metavar='0-255', type=int, nargs=3, help='Red, Green, and Blue color values (0-255).')
    args = parser.parse_args()

    try:
        with LED() as led:
            led.setColor(args.rgb)
            time.sleep(60)
    except:
        led.shutdown()

    GPIO.cleanup()

