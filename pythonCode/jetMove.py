import time
import busio
from board import SCL, SDA
from adafruit_motor import motor
from adafruit_pca9685 import PCA9685

i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)

class carMove(object):
    def __init__(self):
        self.motorL = motor.DCMotor(pca.channels[0], pca.channels[1])
        self.motorR = motor.DCMotor(pca.channels[4], pca.channels[5])
        pca.frequency = 650
        #self.motorL.decay_mode = (motor.SLOW_DECAY)
        #self.motorR.decay_mode = (motor.SLOW_DECAY)
        self.motorL.throttle = 0
        self.motorR.throttle = 0

    def forward(self, speed):
        speed = speed / 100
        self.motorL.throttle = speed
        self.motorR.throttle = speed

    def back(self, speed):
        speed = speed / 100
        self.motorL.throttle = -speed
        self.motorR.throttle = -speed

    def left(self, speed):
        speed = speed / 100
        self.motorL.throttle = -speed
        self.motorR.throttle = speed

    def right(self, speed):
        speed = speed / 100
        self.motorL.throttle = speed
        self.motorR.throttle = -speed

    def track_right(self, delta, rate):
        delta = delta / 100
        self.motorL.throttle = ((0.3 + delta) * rate)
        self.motorR.throttle = (0.3 * rate)

    def track_left(self, delta, rate):
        delta = delta / 100
        self.motorL.throttle = (0.3 * rate)
        self.motorR.throttle = ((0.3 - delta) * rate)

    def little_right(self, delta, rate):
        delta = delta / 100
        self.motorL.throttle = ((0.6 + delta) * rate)
        self.motorR.throttle = (0.6 * rate)

    def little_left(self, delta, rate):
        delta = delta / 100
        self.motorL.throttle = (0.6 * rate)
        self.motorR.throttle = ((0.6 - delta) * rate)

    def brake(self):
        self.motorL.throttle = 0
        self.motorR.throttle = 0

    def forward_turn(self, speed_left, speed_right):
        self.motorL.throttle = speed_left
        self.motorR.throttle = speed_right

    def MotorStop(self):
        self.motorL.throttle = 0
        self.motorR.throttle = 0
