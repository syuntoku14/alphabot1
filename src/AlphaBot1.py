#!/usr/bin/env python
import pigpio

class WheelSetter():
    def __init__(self, pi, left_front=12, left_back=13, right_back=20, right_front=21, enable_left=6, enable_right=26):
        # pin assignment
        self.left_front = left_front
        self.left_back = left_back
        self.right_back = right_back
        self.right_front = right_front
        self.enable_left = enable_left
        self.enable_right = enable_right

        # initialize gpio
        self.pi = pi  # pigpio.pi()
        self.pi.set_mode(self.left_front, pigpio.OUTPUT)
        self.pi.set_mode(self.left_back, pigpio.OUTPUT)
        self.pi.set_mode(self.right_back, pigpio.OUTPUT)
        self.pi.set_mode(self.right_front, pigpio.OUTPUT)
        self.pi.set_mode(self.enable_left, pigpio.OUTPUT)
        self.pi.set_mode(self.enable_right, pigpio.OUTPUT)
        self.set_wheel_speeds(0, 0)

    def vel2duty(self, vel):
        # convert velocity to duty ratio
        duty = int(255 * vel)  # 255 == duty 1
        return duty

    def set_left_speed(self, vel):
        self.pi.set_PWM_dutycycle(self.enable_left, self.vel2duty(vel))

    def set_right_speed(self, vel):
        self.pi.set_PWM_dutycycle(self.enable_right, self.vel2duty(vel))

    def set_wheel_speeds(self, vl, vr):
        # set wheel speeds accordint to vl and vr
        if vl >= 0:
            self.pi.write(self.left_front, 1)
            self.pi.write(self.left_back, 0)
        else:
            self.pi.write(self.left_front, 0)
            self.pi.write(self.left_back, 1)

        if vr >= 0:
            self.pi.write(self.right_front, 1)
            self.pi.write(self.right_back, 0)
        else:
            self.pi.write(self.right_front, 0)
            self.pi.write(self.right_back, 1)
        # set velocities
        self.set_left_speed(vl)
        self.set_right_speed(vr)


class SensorGetter():
    def __init__(self, pi, encoder_left=7, encoder_right=8):
        # photo interupter of encoders
        self.encoder_left = encoder_left
        self.encoder_right = encoder_right

        self.pi = pi  # pigpio.pi()
        self.pi.set_mode(self.encoder_left, pigpio.INPUT)
        self.pi.set_mode(self.encoder_right, pigpio.INPUT)

    def ret_encoder_vals(self):
        l = self.pi.read(self.encoder_left)
        r = self.pi.read(self.encoder_right)
        return l, r
