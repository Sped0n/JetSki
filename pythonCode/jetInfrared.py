# Infrared obstacle avoidance module

import Jetson.GPIO as GPIO
import time

GPIO.setwarnings(False)
GPIO.setmode(GPIO.TEGRA_SOC)


class infrared(object):
    def __init__(self):
        self.GPIO_Infrared_right = "DAP4_SCLK"  # TEGRA_SOC!!!
        self.GPIO_Infrared_left = "UART2_RTS"
        GPIO.setup(self.GPIO_Infrared_right, GPIO.IN)
        GPIO.setup(self.GPIO_Infrared_left, GPIO.IN)

    def infraredMeasure(self):
        left_measure = GPIO.input(self.GPIO_Infrared_left)
        right_measure = GPIO.input(self.GPIO_Infrared_right)
        return [left_measure, right_measure]


if __name__ == '__main__':
    try:
        car = infrared()
        while True:
            [left, right] = car.infraredMeasure()
            print(left, right)
            time.sleep(1)
    except KeyboardInterrupt:
        print("Measurement stopped by User")
        GPIO.cleanup()
