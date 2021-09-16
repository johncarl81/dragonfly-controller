#!/usr/bin/env python
import RPi.GPIO as GPIO
import argparse
import threading
import time

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

        self.rgb = [0, 0, 0]
        self.blink_value = False

        self.shutdownBlink = False
        self.blinkThread = threading.Thread(target=self.blink_operation)
        self.blinkThread.start()

    def blink_operation(self):
        while not self.shutdownBlink:
            if (self.blink_value):
                previousrgb = self.rgb
                self.setColor([0, 0, 0])
                self.update()
                self.rgb = previousrgb
                time.sleep(1)
            self.setColor(self.rgb)
            self.update()
            time.sleep(1)

    def __enter__(self):
        return self

    def __exit__(self):
        self.shutdown()

    def shutdown(self):
        self.shutdownBlink = True
        self.blinkThread.join()
        self.setColor([0, 0, 0])
        self.red.stop()
        self.green.stop()
        self.blue.stop()

    def blink(self):
        self.blink_value = True

    def solid(self):
        self.blink_value = False

    def update(self):
        self.red.ChangeDutyCycle(100 - self.rgb[0])
        self.green.ChangeDutyCycle(100 - self.rgb[1])
        self.blue.ChangeDutyCycle(100 - self.rgb[2])

    # Set a color by giving R, G, and B values of 0-255.
    def setColor(self, rgb=None):
        # Convert 0-255 range to 0-100.
        if rgb is None:
            rgb = []
        self.rgb = [(x / 255.0) * 100 for x in rgb]


if __name__ == '__main__':
    # Get RGB colors from command line arguments.
    parser = argparse.ArgumentParser(description='Light up the world!')
    parser.add_argument('rgb', metavar='0-255', type=int, nargs=3, help='Red, Green, and Blue color values (0-255).')
    args = parser.parse_args()

    try:
        with LED() as led:
            led.setColor(args.rgb)
            time.sleep(60)
    except:
        led.shutdown()

    GPIO.cleanup()
