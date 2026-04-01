import time
from adafruit_servokit import ServoKit


kit = ServoKit(channels=16)
# hinge: 0,1,2,3
# wheel: 6,7

def move_forward():
    kit.continuous_servo[6].throttle = 1
    kit.continuous_servo[7].throttle = -1
    time.sleep(3)
    kit._pca.channels[6].duty_cycle = 0
    kit._pca.channels[7].duty_cycle = 0

def move_backward():
    kit.continuous_servo[6].throttle = -1
    kit.continuous_servo[7].throttle = 1
    time.sleep(3)
    kit._pca.channels[6].duty_cycle = 0
    kit._pca.channels[7].duty_cycle = 0

def turn_left():
    kit.continuous_servo[6].throttle = -1
    kit.continuous_servo[7].throttle = -1
    time.sleep(3)
    kit._pca.channels[6].duty_cycle = 0
    kit._pca.channels[7].duty_cycle = 0

def turn_right():
    kit.continuous_servo[6].throttle = 1
    kit.continuous_servo[7].throttle = 1
    time.sleep(3)
    kit._pca.channels[6].duty_cycle = 0
    kit._pca.channels[7].duty_cycle = 0

def arm_0_test(time_interval):
    kit.continuous_servo[0].throttle = -1
    time.sleep(time_interval)
    kit.continuous_servo[0].throttle = 1
    time.sleep(time_interval)
    kit._pca.channels[0].duty_cycle = 0

def arm_1_test(time_interval):
    kit.continuous_servo[1].throttle = -1
    time.sleep(time_interval)
    kit.continuous_servo[1].throttle = 1
    time.sleep(time_interval)
    kit._pca.channels[1].duty_cycle = 5599
    kit._pca.channels[1].duty_cycle = 5592


def arm_2_test(time_interval):
    kit.continuous_servo[2].throttle = -1
    time.sleep(time_interval)
    kit.continuous_servo[2].throttle = 1
    time.sleep(time_interval)
    kit._pca.channels[2].duty_cycle = 0

def arm_3_test(time_interval):
    kit.continuous_servo[3].throttle = -1
    time.sleep(time_interval)
    kit.continuous_servo[3].throttle = 1
    time.sleep(time_interval)
    kit._pca.channels[3].duty_cycle = 0

def arm_open(arm, time_interval):
    kit.continuous_servo[arm].throttle = -1
    time.sleep(time_interval)
    kit._pca.channels[arm].duty_cycle = 0
    if arm == 1:
        kit._pca.channels[arm].duty_cycle = 5599
        kit._pca.channels[arm].duty_cycle = 5592

def arm_close(arm, time_interval):
    kit.continuous_servo[arm].throttle = 1
    time.sleep(time_interval)
    kit._pca.channels[arm].duty_cycle = 0
    if arm == 1:
        kit._pca.channels[arm].duty_cycle = 5599
        kit._pca.channels[arm].duty_cycle = 5592
