import AlphaBot1
import pigpio

pi = pigpio.pi()
wheel = AlphaBot1.WheelSetter(pi)

wheel.set_wheel_speeds(0, 0)
