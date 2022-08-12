"""
actuators.py
Classes to control the motors and servos. These classes
are wrapped in a mixer class before being used in the drive loop.
"""

import time
import donkeycar as dk

class PCA9685:
    """
    PWM motor controler using PCA9685 boards.
    This is used for most RC Cars
    """
    def __init__(self, channel, frequency=60):
        import Adafruit_PCA9685
        # Initialise the PCA9685 using the default address (0x40).
        self.pwm = Adafruit_PCA9685.PCA9685()
        self.pwm.set_pwm_freq(frequency)
        self.channel = channel

    def set_pulse(self, pulse):
        self.pwm.set_pwm(self.channel, 0, pulse)

    def run(self, pulse):
        self.set_pulse(pulse)

class PWMSteering:
    """
    Wrapper over a PWM motor cotnroller to convert angles to PWM pulses.
    """
    LEFT_ANGLE = -1
    RIGHT_ANGLE = 1

    def __init__(self,left_pulse=60,
                       right_pulse=110):
        #from _XiaoRGEEK_SERVO_ import XR_Servo
        from donkeycar.parts._XiaoRGEEK_SERVO_ import XR_Servo
        self.Servo = XR_Servo()
        self.left_pulse = left_pulse
        self.right_pulse = right_pulse
        self.lastAngle = 85
        self.Servo.XiaoRGEEK_SetServoAngle(1,self.lastAngle)
        self.Servo.XiaoRGEEK_SaveServo();

    def run(self, angle):
        #map absolute angle to angle that vehicle can implement.
        pulse = dk.util.data.map_range(angle,
                                        self.LEFT_ANGLE, self.RIGHT_ANGLE,
                                        self.left_pulse, self.right_pulse)
        #print(pulse)
        if((self.lastAngle-pulse)>1)|((pulse-self.lastAngle)>1):
            self.Servo.XiaoRGEEK_SetServoAngle(1,pulse)
            self.lastAngle=pulse
            

    def shutdown(self):
        self.run(0) #set steering straight



class PWMThrottle:
    """
    Wrapper over a PWM motor cotnroller to convert -1 to 1 throttle
    values to PWM pulses.
    """

    MIN_THROTTLE = -1
    MAX_THROTTLE =  1

    def __init__(self,
                       max_pulse=200,
                       min_pulse=0,
                       zero_pulse=100):

        import RPi.GPIO
        self.GPIO = RPi.GPIO
        self.GPIO.setmode(self.GPIO.BCM)

        ########电机驱动接口定义#################
        self.ENA = 13	#//L298使能A
        self.ENB = 20	#//L298使能B
        self.IN1 = 19	#//电机接口1
        self.IN2 = 16	#//电机接口2
        self.IN3 = 21	#//电机接口3
        self.IN4 = 26	#//电机接口4
        self.GPIO.setwarnings(False)
        #########电机初始化为LOW##########
        self.GPIO.setup(self.ENA,self.GPIO.OUT,initial=self.GPIO.LOW)
        self.ENA_pwm=self.GPIO.PWM(self.ENA,1000) 
        self.ENA_pwm.start(0) 
        self.ENA_pwm.ChangeDutyCycle(100)
        self.GPIO.setup(self.IN1,self.GPIO.OUT,initial=self.GPIO.LOW)
        self.GPIO.setup(self.IN2,self.GPIO.OUT,initial=self.GPIO.LOW)
        self.GPIO.setup(self.ENB,self.GPIO.OUT,initial=self.GPIO.LOW)
        self.ENB_pwm=self.GPIO.PWM(self.ENB,1000) 
        self.ENB_pwm.start(0) 
        self.ENB_pwm.ChangeDutyCycle(100)
        self.GPIO.setup(self.IN3,self.GPIO.OUT,initial=self.GPIO.LOW)
        self.GPIO.setup(self.IN4,self.GPIO.OUT,initial=self.GPIO.LOW)
        self.max_pulse = max_pulse
        self.min_pulse = min_pulse
        self.zero_pulse = zero_pulse
        self.last_pulse = 100
        self.Motor(self.zero_pulse)
        time.sleep(1)

    def Motor_Forward(self):
        self.GPIO.output(self.ENA,True)
        self.GPIO.output(self.ENB,True)
        self.GPIO.output(self.IN1,True)
        self.GPIO.output(self.IN2,False)
        self.GPIO.output(self.IN3,True)
        self.GPIO.output(self.IN4,False)

    def Motor_Backward(self):
        self.GPIO.output(self.ENA,True)
        self.GPIO.output(self.ENB,True)
        self.GPIO.output(self.IN1,False)
        self.GPIO.output(self.IN2,True)
        self.GPIO.output(self.IN3,False)
        self.GPIO.output(self.IN4,True)

    def Motor_TurnLeft(self):
        self.GPIO.output(self.ENA,True)
        self.GPIO.output(self.ENB,True)
        self.GPIO.output(self.IN1,True)
        self.GPIO.output(self.IN2,False)
        self.GPIO.output(self.IN3,False)
        self.GPIO.output(self.IN4,True)

    def Motor_TurnRight(self):
        self.GPIO.output(self.ENA,True)
        self.GPIO.output(self.ENB,True)
        self.GPIO.output(self.IN1,False)
        self.GPIO.output(self.IN2,True)
        self.GPIO.output(self.IN3,True)
        self.GPIO.output(self.IN4,False)

    def Motor_Stop(self):
        self.GPIO.output(self.ENA,False)
        self.GPIO.output(self.ENB,False)
        self.GPIO.output(self.IN1,False)
        self.GPIO.output(self.IN2,False)
        self.GPIO.output(self.IN3,False)
        self.GPIO.output(self.IN4,False)

    def Motor(self,pulse):
        if pulse>=100:
            self.Motor_Forward()
            self.ENA_pwm.ChangeDutyCycle(pulse-100)
            self.ENB_pwm.ChangeDutyCycle(pulse-100)
        else:
            self.Motor_Backward()
            self.ENA_pwm.ChangeDutyCycle(100-pulse)
            self.ENB_pwm.ChangeDutyCycle(100-pulse)
    def run(self, throttle):
        if throttle > 0:
            pulse = dk.util.data.map_range(throttle,
                                                    0, self.MAX_THROTTLE,
                                                    self.zero_pulse, self.max_pulse)
        else:
            pulse = dk.util.data.map_range(throttle,
                                                    self.MIN_THROTTLE, 0,
                                                    self.min_pulse, self.zero_pulse)
        if((self.last_pulse-pulse)>1)|((pulse-self.last_pulse)>1):
            self.Motor(pulse)
            self.last_pulse=pulse
        

    def shutdown(self):
        self.run(0) #stop vehicle



class Adafruit_DCMotor_Hat:
    """
    Adafruit DC Motor Controller
    Used for each motor on a differential drive car.
    """
    def __init__(self, motor_num):
        from Adafruit_MotorHAT import Adafruit_MotorHAT
        import atexit

        self.FORWARD = Adafruit_MotorHAT.FORWARD
        self.BACKWARD = Adafruit_MotorHAT.BACKWARD
        self.mh = Adafruit_MotorHAT(addr=0x60)

        self.motor = self.mh.getMotor(motor_num)
        self.motor_num = motor_num

        atexit.register(self.turn_off_motors)
        self.speed = 0
        self.throttle = 0


    def run(self, speed):
        """
        Update the speed of the motor where 1 is full forward and
        -1 is full backwards.
        """
        if speed > 1 or speed < -1:
            raise ValueError( "Speed must be between 1(forward) and -1(reverse)")

        self.speed = speed
        self.throttle = int(dk.util.data.map_range(abs(speed), -1, 1, -255, 255))

        if speed > 0:
            self.motor.run(self.FORWARD)
        else:
            self.motor.run(self.BACKWARD)
 
        self.motor.setSpeed(self.throttle)


    def shutdown(self):
        self.mh.getMotor(self.motor_num).run(Adafruit_MotorHAT.RELEASE)

class Maestro:
    """
    Pololu Maestro Servo controller
    Use the MaestroControlCenter to set the speed & acceleration values to 0!
    """
    import threading

    maestro_device = None
    astar_device = None
    maestro_lock = threading.Lock()
    astar_lock = threading.Lock()

    def __init__(self, channel, frequency = 60):
        import serial

        if Maestro.maestro_device == None:
            Maestro.maestro_device = serial.Serial('/dev/ttyACM0', 115200)

        self.channel = channel
        self.frequency = frequency
        self.lturn = False
        self.rturn = False
        self.headlights = False
        self.brakelights = False

        if Maestro.astar_device == None:
            Maestro.astar_device = serial.Serial('/dev/ttyACM2', 115200, timeout= 0.01)

    def set_pulse(self, pulse):
        # Recalculate pulse width from the Adafruit values
        w = pulse * (1 / (self.frequency * 4096)) # in seconds
        w *= 1000 * 1000  # in microseconds
        w *= 4  # in quarter microsenconds the maestro wants
        w = int(w)

        with Maestro.maestro_lock:
            Maestro.maestro_device.write(bytearray([ 0x84,
                                                     self.channel,
                                                     (w & 0x7F),
                                                     ((w >> 7) & 0x7F)]))

    def set_turn_left(self, v):
        if self.lturn != v:
            self.lturn = v
            b = bytearray('L' if v else 'l', 'ascii')
            with Maestro.astar_lock:
                Maestro.astar_device.write(b)

    def set_turn_right(self, v):
        if self.rturn != v:
            self.rturn = v
            b = bytearray('R' if v else 'r', 'ascii')
            with Maestro.astar_lock:
                Maestro.astar_device.write(b)

    def set_headlight(self, v):
        if self.headlights != v:
            self.headlights = v
            b = bytearray('H' if v else 'h', 'ascii')
            with Maestro.astar_lock:
                Maestro.astar_device.write(b)

    def set_brake(self, v):
        if self.brakelights != v:
            self.brakelights = v
            b = bytearray('B' if v else 'b', 'ascii')
            with Maestro.astar_lock:
                Maestro.astar_device.write(b)

    def readline(self):
        ret = None
        with Maestro.astar_lock:
            # expecting lines like
            # E n nnn n
            if Maestro.astar_device.inWaiting() > 8:
                ret = Maestro.astar_device.readline()

        if ret != None:
            ret = ret.rstrip()

        return ret

class Teensy:
    """
    Teensy Servo controller
    """
    import threading

    teensy_device = None
    astar_device = None
    teensy_lock = threading.Lock()
    astar_lock = threading.Lock()

    def __init__(self, channel, frequency = 60):
        import serial

        if Teensy.teensy_device == None:
            Teensy.teensy_device = serial.Serial('/dev/teensy', 115200, timeout = 0.01)

        self.channel = channel
        self.frequency = frequency
        self.lturn = False
        self.rturn = False
        self.headlights = False
        self.brakelights = False

        if Teensy.astar_device == None:
            Teensy.astar_device = serial.Serial('/dev/astar', 115200, timeout = 0.01)

    def set_pulse(self, pulse):
        # Recalculate pulse width from the Adafruit values
        w = pulse * (1 / (self.frequency * 4096)) # in seconds
        w *= 1000 * 1000  # in microseconds

        with Teensy.teensy_lock:
            Teensy.teensy_device.write(("%c %.1f\n" % (self.channel, w)).encode('ascii'))

    def set_turn_left(self, v):
        if self.lturn != v:
            self.lturn = v
            b = bytearray('L' if v else 'l', 'ascii')
            with Teensy.astar_lock:
                Teensy.astar_device.write(b)

    def set_turn_right(self, v):
        if self.rturn != v:
            self.rturn = v
            b = bytearray('R' if v else 'r', 'ascii')
            with Teensy.astar_lock:
                Teensy.astar_device.write(b)

    def set_headlight(self, v):
        if self.headlights != v:
            self.headlights = v
            b = bytearray('H' if v else 'h', 'ascii')
            with Teensy.astar_lock:
                Teensy.astar_device.write(b)

    def set_brake(self, v):
        if self.brakelights != v:
            self.brakelights = v
            b = bytearray('B' if v else 'b', 'ascii')
            with Teensy.astar_lock:
                Teensy.astar_device.write(b)

    def teensy_readline(self):
        ret = None
        with Teensy.teensy_lock:
            # expecting lines like
            # E n nnn n
            if Teensy.teensy_device.inWaiting() > 8:
                ret = Teensy.teensy_device.readline()

        if ret != None:
            ret = ret.rstrip()

        return ret

    def astar_readline(self):
        ret = None
        with Teensy.astar_lock:
            # expecting lines like
            # E n nnn n
            if Teensy.astar_device.inWaiting() > 8:
                ret = Teensy.astar_device.readline()

        if ret != None:
            ret = ret.rstrip()

        return ret

class MockController(object):
    def __init__(self):
        pass

    def run(self, pulse):
        pass

    def shutdown(self):
        pass
