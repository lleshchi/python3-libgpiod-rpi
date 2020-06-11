import RPi.GPIO as GPIO
import RPi.GPIO_DEVEL as GPIO_DEVEL
import pytest
import time
import re

def test_init():
    foo = GPIO.PWM(1,2)
