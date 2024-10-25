#!/usr/bin/env python3

import Jetson.GPIO as GPIO

"""
mode = GPIO.getmode()
print("Mode is",mode)


gpio_pin = 27

GPIO.setmode(GPIO.BOARD)

GPIO.setup(gpio_pin,GPIO.OUT)

GPIO.output(gpio_pin, GPIO.HIGH)



F9P = 29
GPIO.setmode(GPIO.BOARD)

GPIO.setup(F9P, GPIO.OUT)
GPIO.output(F9P, GPIO.HIGH)

"""
F9P = 29
GPIO.setmode(GPIO.BOARD)

GPIO.setup(F9P, GPIO.OUT)
while 1:

	GPIO.output(F9P, GPIO.HIGH)

	print(GPIO.input(F9P))
