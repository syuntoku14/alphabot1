import pigpio
import time

class AlphaBot(object):
	
	def __init__(self,in1=12,in2=13,ena=6,in3=20,in4=21,enb=26,
                encl=7, encr=8, redl=19, redr=1):
            # pin assignment
            # wheel pins
	    self.lf = in1  # left front
	    self.lb = in2
	    self.rb = in3
	    self.rf = in4
	    self.ENL = ena
	    self.ENR = enb

            # photo interupter encoder
            self.encl = encl
            self.encr = encr

            # infred sensor
            self.redl = redl
            self.redr = redr

            # initialize gpio
            self.pi = pigpio.pi()
	    self.pi.set_mode(self.lf, pigpio.OUTPUT)
	    self.pi.set_mode(self.lb, pigpio.OUTPUT)
	    self.pi.set_mode(self.rb, pigpio.OUTPUT)
	    self.pi.set_mode(self.rf, pigpio.OUTPUT)
            self.pi.set_mode(self.ENL, pigpio.OUTPUT)
            self.pi.set_mode(self.ENR, pigpio.OUTPUT)
            self.pi.set_mode(self.encl, pigpio.INPUT)
            self.pi.set_mode(self.encr, pigpio.INPUT)
            self.pi.set_mode(self.redl, pigpio.INPUT)
            self.pi.set_mode(self.redr, pigpio.INPUT)

            self.set_wheel_speeds(0, 0)

        def vel2duty(self, vel):
            duty = int(1e6)  # 1e6 == duty 1
            return duty

       	def set_left_speed(self,vel):
            self.pi.hardware_PWM(self.ENL, 100, self.vel2duty(vel))  # 100Hz

       	def set_right_speed(self,vel):
            self.pi.hardware_PWM(self.ENR, 100, self.vel2duty(vel))
		
	def set_wheel_speeds(self, vl, vr):
            # choose the direction of the wheels
            if vl >= 0: 
                self.pi.write(self.lf, 1)
                self.pi.write(self.lb, 0)
            else: 
                self.pi.write(self.lf, 0)
                self.pi.write(self.lb, 1)

            if vr >= 0: 
                self.pi.write(self.rf, 1)
                self.pi.write(self.rb, 0)
            else: 
                self.pi.write(self.rf, 0)
                self.pi.write(self.rb, 1)

            # set velocities
            self.set_left_speed(vl)
            self.set_right_speed(vr)

        def ret_encoder_vals(self):
            l = self.pi.read(self.encl)
            r = self.pi.read(self.encr)
            return l, r
