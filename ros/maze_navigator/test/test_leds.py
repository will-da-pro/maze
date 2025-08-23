import time

import gpiozero


def test_leds():
    red_led = gpiozero.LED(10)
    green_led = gpiozero.LED(11)

    red_led.on()
    time.sleep(2)
    red_led.off()
    time.sleep(2)

    green_led.on()
    time.sleep(2)
    green_led.off()
    time.sleep(2)
