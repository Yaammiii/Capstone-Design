import rclpy
from std_msgs.msg._int8_multi_array import Int8MultiArray
from time import sleep, time
import os
import pigpio


# more info at http://abyz.me.uk/rpi/pigpio/python.html#set_pin_servo_pulsewidth

os.system('sudo pigpiod')
sleep(1)
class StepperMotor:
    def __init__(self, pi, dir_pins, step_pin, enable_pin): # (self, pi, dir_pins, step_pin, mode_pin1, mode_pin2, mode_pin3, enable_pin, sequence):
        self.pi = pi
        self.dir_pin = dir_pins
        self.step_pin = step_pin
        self.enable = enable_pin

        self.pi.set_mode(self.dir_pin, pigpio.OUTPUT)
        self.pi.set_mode(self.step_pin, pigpio.OUTPUT)
        self.pi.set_mode(self.enable, pigpio.OUTPUT)
    
    def enable_mode(self):
        self.pi.write(self.enable, 0)

    def disable_mode(self):    
        self.pi.write(self.enable, 1)
    
    def doClockwiseStep(self, freq):
        self.pi.write(self.dir_pin,0)
        self.pi.hardware_PWM(self.step_pin, 200, 500000)
        sleep(0.1)
        self.pi.hardware_PWM(self.step_pin, 700, 500000)
        sleep(0.1)
        self.pi.hardware_PWM(self.step_pin, freq, 500000)
        sleep(10000)
    
    def doСounterclockwiseStep(self, freq):
        self.pi.write(self.dir_pin, 1)
        self.pi.hardware_PWM(self.step_pin, 200, 500000)
        sleep(0.1)
        self.pi.hardware_PWM(self.step_pin, 700, 500000)
        sleep(0.1)
        self.pi.hardware_PWM(self.step_pin, freq, 500000)
        sleep(10000)

pi = pigpio.pi()

########Dutyratio########
#pin_motor = 18  # 12
motor = StepperMotor(pi=pi,
                     dir_pins=23,
                     step_pin=18,
                     #mode_pin1 = 23,
                     #mode_pin2 = 24,
                     #mode_pin3 = 25,
                     enable_pin = 24
                     #sequence = 'Full'
                    )
pin_servo = 13  # 33
########DC motor Wheel_Dir########
#pi.write(23, 1)
#pi.write(24, 0)
########################
# Servo : 600~2400
# Motor : 15~100 (15~100%)
# servo_angle = (servo_pulse-600)/10
# servo_pulse = servo_angle*10 + 600
# motor_pulse = motor_duty*10000
########################

motor.disable_mode()
print('killed')
motor.enable_mode()

servo_angle = 90
# motor_duty = 0
freq = 1000
start_time = time()
#pi.set_servo_pulsewidth(pin_servo, servo_angle*10 + 600)
#pi.hardware_PWM(pin_motor, 50, 0)

print("pigiod ready")

def body_ctrl(freq, servo_angle):
    try:
        pi.set_servo_pulsewidth(pin_servo, servo_angle*10 + 600)
        motor.doClockwiseStep(freq)
        #pi.hardware_PWM(pin_motor, 50, motor_duty*10000)
        #print('Motor Dutyratio:', motor_duty)
        print('Servo Angle:', servo_angle)
        #print('duty of motor:', pi.get_PWM_dutycycle(pin_motor))
        print()
    except:
        motor.disable_mode()
        pi.set_servo_pulsewidth(pin_servo, servo_angle)


def callback(msg):
    #print('Received message:', end='')
    #print(msg.data)
    body_ctrl(msg.data[0], msg.data[1])
    

def main():
    rclpy.init()
    node = rclpy.create_node('RSPI_node')
    node.create_subscription(Int8MultiArray, 'steer_info', callback, 10)

    while rclpy.ok():
        rclpy.spin_once(node)
        

    node.destroy_node()
    rclpy.shutdown()
    pi.stop()
    os.system('sudo killall pigpiod')
    sleep(1)

if __name__ == '__main__':
    main()
    