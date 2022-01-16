#  Ultrasonic ranging module

import Jetson.GPIO as GPIO
import time

GPIO.setwarnings(False)
GPIO.setmode(GPIO.TEGRA_SOC)


class ultrasound(object):
    def __init__(self):
        self.GPIO_TRIGGER = "LCD_TE"  # TEGRA_SOC!!!
        self.GPIO_ECHO = "SPI2_CS1"
        GPIO.setup(self.GPIO_TRIGGER, GPIO.OUT)  # GPIO input/output definiation
        GPIO.setup(self.GPIO_ECHO, GPIO.IN)
        self.dist_mov_ave = 0

    def DistMeasure(self):  # distance measuing 
        GPIO.output(self.GPIO_TRIGGER, False)
        time.sleep(0.000002)
        GPIO.output(self.GPIO_TRIGGER, True)  # trigger ultrasonic pulse
        time.sleep(0.00001)  # last 10us
        GPIO.output(self.GPIO_TRIGGER, False)  # end the pulse
        ii = 0
        while GPIO.input(self.GPIO_ECHO) == 0:  # when receiving the echo, ECHO will become 1
            ii = ii + 1
            if ii > 10000:
                print('Ultrasound error: the sensor missed the echo')
                return 0
            pass
        start_time = time.time()

        while GPIO.input(
                self.GPIO_ECHO) == 1:  # the duration of high level of ECHO is the time between the emitting the pulse and receiving the echo
            pass
        stop_time = time.time()
        time_elapsed = stop_time - start_time
        distance = (time_elapsed * 34000) / 2
        return distance

    def DistMeasureMovingAverage(self):
        dist_current = self.DistMeasure()
        if dist_current == 0:  # if the sensor missed the echo, the output dis_mov_ave will equal the last dis_mov_ave
            return self.dist_mov_ave
        else:
            self.dist_mov_ave = 0.8 * dist_current + 0.2 * self.dist_mov_ave  # using the moving average of distance measured by sensor to reduce the error
            return self.dist_mov_ave


if __name__ == '__main__':
    try:
        car = ultrasound()
        while True:
            dist = car.DistMeasure()
            print("Measured Distance = {:.2f} cm".format(dist))
            time.sleep(1)
    except KeyboardInterrupt:
        print("Measurement stopped by User")
        GPIO.cleanup()
