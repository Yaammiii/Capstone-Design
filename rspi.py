import rclpy
from std_msgs.msg._int8_multi_array import Int8MultiArray
from time import sleep
import os
import pigpio


# more info at http://abyz.me.uk/rpi/pigpio/python.html#set_pin_servo_pulsewidth

os.system('sudo pigpiod')
sleep(1)
pwm = pigpio.pi()

########Dutyratio########
pin_motor = 18  # 12
pin_servo = 13  # 33
########Wheel_Dir########
pwm.write(23, 1)
pwm.write(24, 0)
########################
# Servo : 600~2400
# Motor : 15~100 (15~100%)
# servo_angle = (servo_pulse-600)/10
# servo_pulse = servo_angle*10 + 600
# motor_pulse = motor_duty*10000
########################

servo_angle = 90
motor_duty = 0

pwm.set_servo_pulsewidth(pin_servo, servo_angle*10 + 600)
pwm.hardware_PWM(pin_motor, 50, 0)

print("pigiod ready")

def body_ctrl(motor_duty, servo_angle):
    pwm.set_servo_pulsewidth(pin_servo, servo_angle*10 + 600)
    pwm.hardware_PWM(pin_motor, 50, motor_duty*10000)
    print('Motor Dutyratio:', motor_duty)
    print('Servo Angle:', servo_angle)
    print('duty of motor:', pwm.get_PWM_dutycycle(pin_motor))
    print()


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
    os.system('sudo killall pigpiod')
    sleep(1)

if __name__ == '__main__':
    main()
    