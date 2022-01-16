import Jetson.GPIO as GPIO
import time
import cv2
import numpy as np
from jetInfrared import infrared
from jetMove import carMove
import busio
from board import SCL, SDA
from adafruit_motor import motor
from adafruit_pca9685 import PCA9685
from threading import Thread

colorY = 'yellow'
colorR = 'red'
colorRP = 'red1'
colorG = 'green'
colorB = 'blue'

i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)
GPIO.setwarnings(False)
GPIO.setmode(GPIO.TEGRA_SOC)
global color_dist
color_dist = {'red': {'Lower': np.array([0, 60, 60]), 'Upper': np.array([7, 255, 255])},
              'red1': {'Lower': np.array([156, 60, 60]), 'Upper': np.array([157, 255, 255])},
              'blue': {'Lower': np.array([86, 80, 46]), 'Upper': np.array([110, 255, 255])},
              'green': {'Lower': np.array([35, 43, 35]), 'Upper': np.array([85, 255, 255])},
              'yellow': {'Lower':np.array([20, 110, 85]), 'Upper': np.array([35, 200, 185])}
              }

class Car(carMove, infrared):
    def __init__(self):
        carMove.__init__(self)
        infrared.__init__(self)

    def AllStop(self):
        carMove.MotorStop(self)
        GPIO.cleanup()
        pca.deinit()

car = Car()
route = 1
cap = cv2.VideoCapture(1) # Two usb cameras.
capD = cv2.VideoCapture(0)

def main_program():
    global colorY, route, car, cap
    center = 320
    ynum = 0 # Number of yellow squares identified in a period of time.
    st = time.time()
    y_st = 0
    y_trig = 0
    while (1):
        if st - time.time() > 1.5:
            ynum = 0
            st = time.time()
        # Zero out the ynum every 1.3 seconds, with the aim of recognizing it several times in 1.3 seconds before it counts
        [left_measure, right_measure] = car.InfraredMeasure()
        # Bypassing obstacles
        if left_measure == 0 and right_measure == 0:
            print("Left routing")
            car.back(50)
            time.sleep(0.3)
            car.left(50)
            time.sleep(0.35)

            car.forward(45)
            time.sleep(1)

            car.right(50)
            time.sleep(0.35)

            car.forward(50)
            time.sleep(0.45)

            car.right(50)
            time.sleep(0.53)

            car.forward(60)
            time.sleep(0.9)
            car.left(80)
            time.sleep(0.38)

        # line tracking
       
        if route == 1: # When route is 1, enable line tracking
            ret, frame = cap.read()
            # img = cv2.flip(frame, -1)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            retval, dst = cv2.threshold(gray, 0, 255, cv2.THRESH_OTSU)
            dst = cv2.dilate(dst, None, iterations=2)
            # dst = cv2.erode(dst, None, iterations=6)
            # cv2.imshow("dst", dst)
            color = dst[380]
            black_count = np.sum(color == 0)
            black_index = np.where(color == 0)
            if black_count == 0:
                black_count = 2
                car.back(40)
                print("zero black back")
                # print("blackcount:", black_count)
                center = 0
            else:
                center = (black_index[0][black_count - 1] + black_index[0][0]) / 2
            direction = center - 320
            # print("direction", direction)
            if abs(direction) > 300:
                car.back(40)
                print("direction back")
            elif direction >= 0:
                if direction <= 28:
                    car.forward(100)
                    print("forward")
                elif direction <= 40 and direction > 28:
                    car.little_right(direction, 1)
                    print("little right")
                else:
                    if direction > 70:
                        direction = 70
                    car.track_right(direction, 0.7)
                    print("right")
            elif direction < -0:
                if direction >= -28:
                    car.forward(100)
                    print("forward")
                elif direction >= -40 and direction < -28:
                    car.little_left(direction, 1)
                    print("little left")
                else:
                    if direction < -70:
                        direction = -70
                    car.track_left(direction, 0.7)
                    print("left")
        else:
            car.brake()
            print("route brake")
        # Identify the yellow square at the end to park
        gs_frame = cv2.GaussianBlur(frame, (5, 5), 0)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        hsv = cv2.erode(hsv, None, iterations=2)
        OcolorY_hsv = cv2.inRange(hsv, color_dist[colorY]['Lower'], color_dist[colorY]['Upper'])
        colorY_hsv = cv2.erode(OcolorY_hsv, None, iterations=8)
        Ycnts = cv2.findContours(colorY_hsv.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        if Ycnts:
            maxcnt = max(Ycnts, key=cv2.contourArea)
            rect = cv2.minAreaRect(maxcnt)
            box = cv2.boxPoints(rect)
            area = cv2.contourArea(box)
            h = abs(box[3, 1] - box[1, 1])
            w = abs(box[3, 0] - box[1, 0])
            arr = np.array([h, w])
            yrate = np.std(arr, ddof=1)
            if yrate < 60:
                if w * h > 7000:
                    ynum += 1
            if ynum == 4:
                y_st = time.time()
                print("yyes") # Yellow square identified
            if time.time() - y_st > 0.3 and y_st != 0: # Prevent two consecutive identifications in a short period of time.
                y_trig = 1
            if y_trig == 1:
                print("all stop")
                car.little_left(-30,1)
                time.sleep(0.617)
                car.brake()
                time.sleep(21)
                y_trig = 0
                y_st = 0
                ynum = 0
                
def others():
    global colorR, colorRP, colorG, colorB, capD, route
    rnum, gnum, bnum = 0 #
    st = time.time()
    y_trig, r_trig, g_trig, b_trig = 0
    regG, regR = 0
    blast, glast, rlast = 0
    # rnum is the cumulative count of the red signal in 1.3 seconds, st is used to reset every 1,3 seconds,
    #  r_st is to go a small delay code before stopping, r_trig is to trigger the stop.
    circularity = 70 # Hough circle parameters
    hough_params_2 = 18
    while(1):
        if st - time.time() > 1.5: # Reset every 1.5 seconds.
            rnum, gnum, bnum = 0 # The number of circles of the corresponding color identified in a period of time.
            st = time.time()
        ret1, frameD = capD.read()
        frameD = cv2.flip(frameD , -1)
        gs_frameD = cv2.GaussianBlur(frameD, (5, 5), 0)
        # dst = cv2.pyrMeanShiftFiltering(frameD, 10, 100)
        grey = cv2.cvtColor(gs_frameD, cv2.COLOR_BGR2GRAY)
        hsv = cv2.cvtColor(frameD, cv2.COLOR_BGR2HSV)
        hsv = cv2.erode(hsv, None, iterations=2)
        # OcolorY_hsv = cv2.inRange(hsv, color_dist[colorY]['Lower'], color_dist[colorY]['Upper'])
        # colorY_hsv = cv2.erode(OcolorY_hsv, None, iterations=8)
        # Ycnts = cv2.findContours(colorY_hsv.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        # colorB_hsv = cv2.inRange(hsv, color_dist[colorB]['Lower'], color_dist[colorB]['Upper'])
        # colorG_hsv = cv2.inRange(hsv, color_dist[colorG]['Lower'], color_dist[colorG]['Upper'])
        # colorR_hsv = cv2.inRange(hsv, color_dist[colorR]['Lower'], color_dist[colorR]['Upper']) + cv2.inRange(hsv, color_dist[colorRP]['Lower'], color_dist[colorRP]['Upper'])
        circles = cv2.HoughCircles(grey, cv2.HOUGH_GRADIENT, 1, circularity, param1=100, param2=hough_params_2,minRadius=5, maxRadius=40)

        if circles is not None:
            x = circles[0][:, 0].astype(int)
            y = circles[0][:, 1].astype(int)
            r = circles[0][:, 2].astype(int)
            num = circles[0].shape[0]
            rate = np.zeros(num)
            for i in range(num):  # traverse all detected circles
                detect_area = (
                    hsv[y[i] - r[i]: y[i] + r[i], x[i] - r[i]:x[i] + r[i]])  # A square in the detected circle
                height, width, channel = detect_area.shape
                # red filter
                if height != 0 and width != 0:
                    if regR == 1:
                        red_mask = cv2.inRange(detect_area, color_dist[colorR]['Lower'],
                                               color_dist[colorR]['Upper']) + cv2.inRange(detect_area,color_dist[colorRP]['Lower'],color_dist[colorRP]['Upper'])
                        red_num_point = np.sum(red_mask / 255)
                        Rrate = red_num_point / (height * width)
                        if Rrate > 0.35:
                            #cv2.circle(frameD, (x[i], y[i]), r[i], (255, 0, 0), 2)
                            #cv2.circle(frameD, (x[i], y[i]), 2, (255, 255, 0), 3)
                            #print("Red", r[i])
                            # print(Rrate)
                            if r[i] >= 14:
                                rnum += 1
                                print("red", r[i])
                    # green filter
                    if regG == 1:
                        green_mask = cv2.inRange(detect_area, color_dist[colorG]['Lower'], color_dist[colorG]['Upper'])
                        green_num_point = np.sum(green_mask / 255)
                        Grate = green_num_point / (height * width)
                        if Grate > 0.4:
                            #cv2.circle(frameD, (x[i], y[i]), r[i], (0, 0, 255), 2)
                            #cv2.circle(frameD, (x[i], y[i]), 2, (0, 0, 255), 3)
                            #print("Green", r[i])
                            if r[i] > 15:
                                gnum += 1
                                print("green", r[i])
                    # blue filter
                    blue_mask = cv2.inRange(detect_area, color_dist[colorB]['Lower'], color_dist[colorB]['Upper'])
                    blue_num_point = np.sum(blue_mask / 255)
                    Brate = blue_num_point / (height * width)
                    if Brate > 0.15:
                        #cv2.circle(frameD, (x[i], y[i]), r[i], (0, 255, 0), 2)
                        #cv2.circle(frameD, (x[i], y[i]), 2, (0, 255, 0), 3)
                        if r[i] >= 26:
                            bnum += 1
                            print("Blue", r[i])
        # cv2.imshow('camera', frameD)
        # cv2.imshow('blue', colorB_hsv)
        # cv2.imshow('green', colorG_hsv)
        # cv2.imshow('RED', colorR_hsv)
        # cv2.imshow('yellow', OcolorY_hsv)
        # cv2.waitKey(1)

        if bnum == 6:
            b_trig = 1
            print("bbbbbbbbbbb",bnum)
            bnum = 0
        else:
            b_trig = 0
        if b_trig == 1 and time.time() - blast > 1:
            time.sleep(0.35)
            print("bstop")
            route = 0
            print("brake")
            time.sleep(17) # Blue light stop 15 seconds, no more line tracking.
            b_trig = 0
            blast = time.time()
            route = 1
            bnum = 0
            b_trig = 0

        if rnum == 4:
            r_trig = 1
            print("rrrrrrrrrrrrrr",rnum)
        if r_trig == 1 and time.time() - rlast > 2:
            time.sleep(0.82)
            route = 0 # After the red light stop dead no longer perform line tracking.
            regG = 1 # After the red light, turn on the green light recognition.
            regR = 0 # Turn off the red light recognition.
            if regG != 1:
                print("rstop")
            rlast = time.time() # Set rlast is the same as the loo'ji of identifying yellow squares above
            # to prevent continuous identification in a short period of time.

        if gnum == 4 and time.time() - glast > 2:
            g_trig = 1
            print("ggggggggggg",gnum)
        if g_trig == 1:
            r_trig = 0
            rnum = 0
            route = 1 # Start line tracking after green light.
            glast = time.time()
            regG = 0 # Because there is only one traffic light in the track, regR will not be set to 1 after this green light is recognized
            g_trig = 0
            gnum = 0
            print("greenpass")

if __name__ == '__main__':
    try:
        t1 = Thread(target = main_program)
        t2 = Thread(target = others)
        t1.start()
        t2.start()

        # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("Measurement stopped by User")
        car.brake()
        car.AllStop()
        cap.release()
        capD.release()
        cv2.destroyAllWindows()


