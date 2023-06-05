import lane_detection as ld
import lidar_scan as ls
import object_scan as _os
####################################
import cv2
import numpy as np
import matplotlib.pyplot as plt
from time import sleep
import time
import random
from ultralytics import YOLO
from multiprocessing import Process, Queue
import rclpy
from rclpy.node import Node
from std_msgs.msg._u_int8_multi_array import UInt8MultiArray
from std_msgs.msg._float32_multi_array import Float32MultiArray
from rclpy.qos import qos_profile_sensor_data, QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image as msg_Image, LaserScan



is_lidar_triggerred = True

class RSPi_Subsriber(Node):
    def __init__(self, 
                 node_name="RSPiSub_Node", 
                 topic_name='Lane_Img'):
        super().__init__(node_name)
        qos_profile = QoSProfile(
            depth = 10,
            history = HistoryPolicy.KEEP_LAST,
            reliability = ReliabilityPolicy.BEST_EFFORT,
            durability = DurabilityPolicy.VOLATILE
        )
        self.count = 1

        # You can create several subscription in single node
        self.subscription1 = self.create_subscription(
                                    msg_Image,
                                    topic_name,
                                    self.listener_callback,
                                    qos_profile)
        self.subscription1
        time.sleep(2)   # skipping sleep may cause the error
    
    def get_data(self):
        rclpy.spin_once(self)
        return self.data

    def listener_callback(self, msg):
        #self.get_logger().info(str(self.count) + 'recieved')
        self.count += 1
        self.data = msg

def send_ctrl_info(
        buffer_Handle,      # Handling data from Lane Detection
        buffer_Slowdown,    # Slowdown data from Yolo
        buffer_Brake_Yolo,  # Braking data from Yolo
        buffer_Brake_Lidar  # braking data from Lidar
    ):
    global is_lidar_triggerred
    rclpy.init()
    node = rclpy.create_node('PC_node')
    publisher = node.create_publisher(Float32MultiArray, 'steer_info', 10)
    msg = Float32MultiArray()
    servo_angle = 90
    motor_duty = 5
    s_time = time.time()
    

    while True:
        motor_duty = 15.5
        if not buffer_Handle.empty():
            servo_angle = buffer_Handle.get()
        if abs(servo_angle-90)> 2:
            motor_duty = 16.1
        if abs(servo_angle-90)>= 5:
            motor_duty = 16.8
        if abs(servo_angle-90)> 7:
            motor_duty = 18
        if abs(servo_angle-90)> 10:
            motor_duty = 21
        if abs(servo_angle-90)>= 15:
            motor_duty = 25
        if abs(servo_angle-90)> 18:
            motor_duty = 28
        if abs(servo_angle-90)> 20:
            motor_duty = 30
        
        if servo_angle <= 60:
            servo_angle =60
        if servo_angle >= 120:
            servo_angle = 120
        #motor_duty=0

        if round(3*time.time()) %3 ==0:
            motor_duty = 5
        
        if not buffer_Brake_Lidar.empty():
            if buffer_Brake_Lidar.get():
                s_time = time.time()

        
        
        if not buffer_Brake_Yolo.empty():
            if buffer_Brake_Yolo.get():
                s_time = time.time()

        if time.time()-s_time<2:
            motor_duty=0
        
        msg.data = [float(motor_duty), float(servo_angle)]
        publisher.publish(msg)
        sleep(0.1)
        print('msg sent:', [motor_duty, servo_angle])

def lane(region, buffer_Handle, buffer_Front_Img):
    rclpy.init(args=None)

    sub = RSPi_Subsriber(node_name = 'RSPiSub_Node',
                        topic_name = 'Lane_Img'
                        )
    print('Start Subscribing')

    curvature_L, curvature_R = 0, 0
    left_fit_L = [2.60030797e-05, -1.62531407e-01,  4.67357700e+02]
    left_fit_R = [2.60030797e-05, -1.62531407e-01,  4.67357700e+02]

    prev_L1, prev_L2, prev_L3, prev_L4 = 0, 0, 0, 90
    prev_R1, prev_R2, prev_R3, prev_R4 = 0, 0, 0, 90

    #capture1 = cv2.VideoCapture(2)
    #capture1.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    #capture1.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    #capture2 = cv2.VideoCapture(0)
    #capture2.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    #capture2.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

    while True:
        #_, frame_L = capture1.read()
        #_, frame_R = capture2.read()

        received_msg = sub.get_data()
        img = np.array(received_msg.data)
        merged_img = np.reshape(img, (240, 960, 3))
        frame_Front = merged_img[:, 0:320, :]
        frame_L = merged_img[:, 320:640, :]
        frame_R = merged_img[:, 640:960, :]


        if buffer_Front_Img.full():
            buffer_Front_Img.get()
        buffer_Front_Img.put(frame_Front)

        frame_L = cv2.rotate(frame_L, cv2.ROTATE_90_COUNTERCLOCKWISE)
        frame_R = cv2.rotate(frame_R, cv2.ROTATE_90_CLOCKWISE)
        prev_L1, prev_R1 = frame_L, frame_R

        try:

            img_L, curvature_L, left_fit_L, angle_L = ld.pipeline(frame_L, region, curvature_L, left_fit_L, 'L')
        except:
            print('Left side Pipeline error')
            img_L, curvature_L, left_fit_L, angle_L = prev_L1, prev_L2, prev_L3, prev_L4
            curvature_L=90
        
        try:
            img_R, curvature_R, left_fit_R, angle_R = ld.pipeline(frame_R, region, curvature_R, left_fit_R,'R')
        except:
            print('Right side PipeLine error')
            img_R, curvature_R, left_fit_R, angle_R = prev_R1, prev_R2, prev_R3, prev_R4
            curvature_R=90
        print('img_L:', round(curvature_L, 1), end='\t')
        print('img_R:', round(curvature_R, 1))
        prev_L1, prev_L2, prev_L3, prev_L4 = img_L, curvature_L, left_fit_L, angle_L
        prev_R1, prev_R2, prev_R3, prev_R4 = img_R, curvature_R, left_fit_R, angle_R

        img_LR = cv2.hconcat([img_L, img_R])
        img_LR = cv2.resize(img_LR, dsize=(960, 720))

        cv2.imshow('Lane Detect', img_LR)
        cv2.waitKey(1)

        if curvature_L == 0:
            curvature = curvature_R
        elif curvature_R == 0:
            curvature = curvature_L
        else:
            curvature = (curvature_L + curvature_R)/2
        #servo_angle = int(180 - (angle_L+angle_R) + 90)
        servo_angle = int(curvature)
        #print(servo_angle)

        if buffer_Handle.full():
            buffer_Handle.get()
        buffer_Handle.put(servo_angle)
        #print("Put Handle Angle data to buffer")


def obj_detection(buffer_Front_Img, buffer_Brake_Yolo, buffer_Slowdown):
    model = YOLO(r"/media/hanwool/USB/integrate1/integrate/best_25.pt")
    class_list = list(model.names.values())
    print('model Loaded')

    # Generate random colors for each class
    detection_colors = []
    for _ in range(len(class_list)):
        r = random.randint(0,255)
        g = random.randint(0,255)
        b = random.randint(0,255)
        detection_colors.append((b,g,r))

    #capture3 = cv2.VideoCapture(0)
    #capture3.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    #capture3.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    # Variable to check stop for a moment
    brake_time, is_stopped = time.time(), False

    while True:
        if buffer_Front_Img.empty():
            continue
        frame = buffer_Front_Img.get()
        data_dict, frame = _os.object_detect(model, 
                                             frame, 
                                             detection_colors, 
                                             class_list)
        cv2.imshow('ObjectDetection', frame)
        detected_obj = [key for key in data_dict.keys()]
        if not is_stopped:
            brake_time = time.time()
        if len(detected_obj) == 0:
            pass
        else:
            for obj in detected_obj:
                if obj == 'trafficlight':
                    if buffer_Brake_Yolo.full():
                            buffer_Brake_Yolo.get()
                    if data_dict[obj]:
                        buffer_Brake_Yolo.put(True)
                    else:
                        buffer_Brake_Yolo.put(False)
                elif obj == 'stop' or 'crosswalk':
                    if buffer_Brake_Yolo.full():
                            buffer_Brake_Yolo.get()
                    # if it triggers to stop for the first time
                    if (not is_stopped) and data_dict[obj]:
                        is_stopped = True
                        buffer_Brake_Yolo.put(True)
                    elif is_stopped and (time.time()-brake_time)<1.5:
                        if buffer_Brake_Yolo.full():
                            buffer_Brake_Yolo.get()
                        buffer_Brake_Yolo.put(True)
                    else:
                        if buffer_Brake_Yolo.full():
                            buffer_Brake_Yolo.get()
                        buffer_Brake_Yolo.put(False)
                        is_stopped = False
                elif obj == 'person':
                    if buffer_Brake_Yolo.full():
                            buffer_Brake_Yolo.get()
                    if data_dict[obj]:
                        buffer_Brake_Yolo.put(True)
                    else:
                        buffer_Brake_Yolo.put(False)
                elif obj == 'speedlimit':
                    if buffer_Slowdown.full():
                            buffer_Slowdown.get()
                    if data_dict[obj]:
                        buffer_Slowdown.put(True)
                    else:
                        buffer_Slowdown.put(False)
        cv2.waitKey(1)


def lidar_info(buffer_Brake_Lidar):
    def callback(msg):
        x, y, is_true = ls.draw(msg)
        plt.figure(1)
        plt.cla()
        plt.ylim(-500,500)
        plt.xlim(-500,500)
        plt.scatter(x, y, c='r', s=13)
        plt.title('Lidar Warning - Hanwool Lee')
        #print(q)
        #plt.scatter(q, r, c='b', s=13)
        plt.pause(0.001)
        #plt.close("all")
        if buffer_Brake_Lidar.full():
            buffer_Brake_Lidar.get()
        buffer_Brake_Lidar.put(is_true)

    rclpy.init()
    node = rclpy.create_node('LiDAR_Sub')
    node.create_subscription(LaserScan, '/scan', callback, qos_profile_sensor_data)
    # Wait for messages to arrive
    while rclpy.ok():
        rclpy.spin_once(node)
if __name__ == "__main__":
    # line detecting area
    top_left = [0, 0]
    top_right = [240, 0]
    bottom_right = [240, 320]
    bottom_left = [0, 320]
    region = (top_left, top_right, bottom_left, bottom_right)

    buffer_Brake_Yolo = Queue(maxsize=5)
    buffer_Brake_Lidar = Queue(maxsize=5)
    buffer_Slowdown = Queue(maxsize=5)
    buffer_Handle = Queue(maxsize=5)
    buffer_Front_Img = Queue(maxsize=5)

    servo_angle = 90
    motor_duty = 0
    buffer_Handle.put(90)

    # process spawning
    p_Lane = Process(target=lane, args=(region, buffer_Handle, buffer_Front_Img,))
    p_Yolo = Process(target=obj_detection, args=(buffer_Front_Img, buffer_Brake_Yolo, buffer_Slowdown,))
    #p_Lidar= Process(target=lidar_info, args=(buffer_Brake_Lidar,))
    p_Ctrl = Process(
        target=send_ctrl_info, 
        args=(buffer_Handle,        # Handling data from Lane Detection
            buffer_Slowdown,        # Slowdown data from Yolo
            buffer_Brake_Yolo,      # Braking data from Yolo
            buffer_Brake_Lidar      # braking data from Lidar
            ,))

    p_Lane.start()
    p_Ctrl.start()
    p_Yolo.start()
    #p_Lidar.start()
    try:
        while True:
            pass

    except:
        rclpy.shutdown()  
        p_Lane.kill()
        p_Ctrl.kill()
        #p_Lidar.kill()
        p_Yolo.kill()
       
