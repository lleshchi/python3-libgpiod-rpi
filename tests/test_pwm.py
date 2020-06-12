import RPi.GPIO as GPIO
import RPi.GPIO_DEVEL as GPIO_DEVEL
import pytest
import time
import re

def test_init():
    GPIO_DEVEL.Reset()
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(18, GPIO.OUT)
    foo = GPIO.PWM(18, 1)

    with pytest.raises(ValueError) as e:
        bar = GPIO.PWM(54, 1)
    assert "channel sent is invalid" in str(e.value)

    with pytest.raises(ValueError) as e:
        bar = GPIO.PWM(-1, 1)
    assert "channel sent is invalid" in str(e.value)

    with pytest.raises(RuntimeError) as e:
        bar = GPIO.PWM(18, 1)
    assert "object already exists" in str(e.value)

    with pytest.raises(ValueError) as e:
        GPIO.setup(19, GPIO.OUT)
        bar = GPIO.PWM(19, -1)
    assert "greater than 0.0" in str(e.value)


def test_start_stop():
    GPIO_DEVEL.Reset()
    GPIO_DEVEL.setdebuginfo(True)
    GPIO.setmode(GPIO.BCM)

    GPIO.setup(18, GPIO.OUT)
    foo = GPIO.PWM(18, 1)

    foo.start(50)
    time.sleep(1)
    foo.stop()

def test_change_attributes():

    GPIO_DEVEL.Reset()
    GPIO_DEVEL.setdebuginfo(True)
    GPIO.setmode(GPIO.BCM)

    GPIO.setup(18, GPIO.OUT)
    foo = GPIO.PWM(18, 1)

    foo.start(50)
    time.sleep(1)

    foo.ChangeFrequency(100)
    foo.ChangeDutyCycle(100)

    with pytest.raises(ValueError) as e:
        foo.ChangeFrequency(-666)
    assert "greater than 0.0" in str(e.value)

    with pytest.raises(ValueError) as e:
        foo.ChangeDutyCycle(-666)
    assert "from 0.0 to 100.0" in str(e.value)

    with pytest.raises(ValueError) as e:
        foo.ChangeDutyCycle(666)
    assert "from 0.0 to 100.0" in str(e.value)

    time.sleep(3)
    foo.stop()
