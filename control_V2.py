import lane_detection as ld
import lidar_scan as ls
import object_scan as _os

####################################
import cv2
import numpy as np
from time import sleep
import time
import random
from multiprocessing import Process, Queue
import matplotlib.pyplot as plt
import warnings
warnings.simplefilter('ignore', np.RankWarning)

############## ROS2 ##################
from cv_bridge import CvBridge
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg._u_int8_multi_array import UInt8MultiArray
from std_msgs.msg._float32_multi_array import Float32MultiArray
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan, CompressedImage

############## YOLO ##################
from ultralytics import YOLO, utils
# utils.TQDM(disable=True)


bridge = CvBridge()
qos_profile = QoSProfile(
            depth = 5,
            history = HistoryPolicy.KEEP_LAST,
            reliability = ReliabilityPolicy.BEST_EFFORT,
            durability = DurabilityPolicy.VOLATILE)

########################################################################
class Lane_Subscriber(Node):
    def __init__(self, thread_num, topic_name, q):
        super().__init__('Image_Subscriber_'+str(thread_num))
        self.q = q
        self.topic_name = topic_name

        # You can create several subscription in single node
        self.subscription = self.create_subscription(
            CompressedImage,
            topic_name,
            self.listener_callback,
            qos_profile)
        time.sleep(1)   # skipping sleep may cause the error

    def listener_callback(self, img_msg):
        frame = bridge.compressed_imgmsg_to_cv2(img_msg)
        if self.q.full():self.q.get()
        frame = cv2.rotate(frame, cv2.ROTATE_180)
        self.q.put(frame)

        # self.get_logger().info(f'{self.topic_name} Received: {header}')
########################################################################
class Lidar_Subscriber(Node):
    def __init__(self, buffer_Brake_Lidar, buffer_plot_Lidar, trigger_distance):
        super().__init__('Lidar_Subscriber')
        self.buffer_Brake_Lidar = buffer_Brake_Lidar
        self.buffer_plot_Lidar = buffer_plot_Lidar
        self.trigger_distance = trigger_distance

        # You can create several subscription in single node
        self.subscription = self.create_subscription(
            LaserScan, 
            '/scan', 
            self.listener_callback, 
            qos_profile)
        time.sleep(1)   # skipping sleep may cause the error

    def listener_callback(self, msg):
        x, y, is_true = ls.draw(msg, trigger_distance=self.trigger_distance)
        self.buffer_plot_Lidar.put([x,y])

        if self.buffer_Brake_Lidar.full():
            self.buffer_Brake_Lidar.get()
        self.buffer_Brake_Lidar.put(is_true)


########################################################################
class CTRL_Publisher(Node):
    def __init__(self, buffer_Handle):
        super().__init__('CTRL_Pub_Node')
        self.buffer_Handle = buffer_Handle
        
        self.msg = UInt8MultiArray()

        # You can create several subscription in single node
        self.publisher = self.create_publisher(
            msg_type=UInt8MultiArray,
            topic='steer_info',
            qos_profile=qos_profile)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        if self.buffer_Handle.empty():
            # self.msg.data = [int(0), int(90)]
            # self.publisher.publish(self.msg)
            return False
        steer_info = self.buffer_Handle.get()
        a = steer_info[0]
        b = steer_info[1]
        self.msg.data = [int(a),int(b)]
        self.publisher.publish(self.msg)
        # self.get_logger().info(f'CTRL info sent: {[steer_info[0], steer_info[1]]}')
########################################################################
def obj_detection(buffer_Front_Img, buffer_Brake_Yolo, buffer_Slowdown):
    model = YOLO(r"/home/hanwool/Desktop/haksulje/run/Final/weights/0.990.pt")
    class_list = list(model.names.values())
    print('model Loaded')

    # Generate random colors for each class
    detection_colors = []
    for _ in range(len(class_list)):
        r = random.randint(0,255)
        g = random.randint(0,255)
        b = random.randint(0,255)
        detection_colors.append((b,g,r))

    ip = 'http://192.168.0.202:81/stream'

    capture3 = cv2.VideoCapture(ip)
    # capture3.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    # capture3.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

    brake_time, is_stopped = time.time(), False

    while True:
        ret, frame = capture3.read()
        # frame = cv2.resize(frame, (640, 480))
        frame = cv2.rotate(frame, cv2.ROTATE_180)


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

def lane(region, img_q, which_lane, curvature_q, laneimg_q):
    
    curvature = 0
    left_fit = [2.60030797e-05, -1.62531407e-01,  4.67357700e+02]

    while True:
        if not img_q.empty():
            frame = img_q.get()
            # print(f'{which_lane}-Lane Process NOT passed')
        else:
            # print(f'{which_lane}-Lane Process passed')
            continue

        #try:
        # cv2.imshow(f'unprocessed{which_lane}', frame)
        laned_img, curvature, left_fit, angle, fitcr = ld.pipeline(frame, region, curvature, left_fit, which_lane)
        cv2.waitKey(1)
        # print(f'img_{which_lane}:', round(curvature, 1))
        if curvature_q.full():
            curvature_q.get()
        curvature_q.put([curvature, fitcr])
        
        if laneimg_q.full():
            laneimg_q.get()
        laneimg_q.put(laned_img)
            
        # except:
        #     print(f'{which_lane} side Pipeline error')

def merge_lane_info(
        buffer_Handle,      # Handling data from Lane Detection
        buffer_Slowdown,    # Slowdown data from Yolo
        buffer_Brake_Yolo,  # Braking data from Yolo
        buffer_Brake_Lidar,  # braking data from Lidar
        L_curvature,
        L_laneimg,
        R_curvature,
        R_laneimg 
    ):

    servo_angle = 90
    s_time = time.time()
    
    while True:
        if (L_curvature.empty() or R_curvature.empty()):
            continue
        angle_L, dist_L = L_curvature.get()
        angle_R, dist_R= R_curvature.get()
        print(f'distL:{dist_L} distR:{dist_R}')

        if angle_L==-1 and angle_R==-1:
            continue
        elif angle_L == -1:
            servo_angle = int(90*0.15 + (angle_R)*0.85)
        elif angle_R == -1:
            servo_angle = int(90*0.15 + (angle_L)*0.85)
        else:
            servo_angle = int(90*0.57 + ((angle_L+angle_R)/2)*0.43)
        # print('servo_angle:', servo_angle)
        
        # if vehicle get close to left lane
        if 84<=servo_angle<=96 and dist_L>348:
            servo_angle -= 15
            print('laned adjusted')
        
        # elif 84<=servo_angle<=96 and dist_L<150:
        #     servo_angle += 4
        #     print('laned adjusted')

        # if vehicle get close to right lane    
        # elif 84<=servo_angle<=96 and dist_R>390:
        #     servo_angle -= 4
        #     print('lane adjusted')
        
        elif 84<=servo_angle<=96 and dist_R<340:
            servo_angle += 15
            print('lane adjusted')

        if servo_angle <= 50:
            servo_angle =50
        if servo_angle >= 130:
            servo_angle = 130

        motor_speed = 45

        # if abs(servo_angle-90)> 2:
        #     motor_speed = 50
        # elif abs(servo_angle-90)>= 5:
        #     motor_speed = 55
        # elif abs(servo_angle-90)> 7:
        #     motor_speed = 60
        if abs(servo_angle-90)> 10:
            motor_speed -= 10
        elif abs(servo_angle-90)>= 15:
            motor_speed -= 20
        # elif abs(servo_angle-90)> 18:
        #     motor_speed = 75
        # elif abs(servo_angle-90)> 20:
        #     motor_speed = 80
        motor_speed += 25

        #######################################3c
        

        if not buffer_Brake_Lidar.empty():
            if buffer_Brake_Lidar.get():
                s_time = time.time()
        
        if not buffer_Brake_Yolo.empty():
            if buffer_Brake_Yolo.get():
                s_time = time.time()
        
        if not buffer_Slowdown.empty():
            if buffer_Slowdown.get():
                motor_speed-=30

        if time.time()-s_time<2:
            motor_speed=0

        if buffer_Handle.full():
            buffer_Handle.get()
        buffer_Handle.put([int(servo_angle), int(motor_speed)])
        if (L_laneimg.empty() or R_laneimg.empty()):
            continue
        img_L = L_laneimg.get()
        img_R = R_laneimg.get()
        merged_img = cv2.hconcat([img_L, img_R])

        print('servo:', servo_angle, '\tspeed:', motor_speed)
        cv2.imshow('Lane_IMG', merged_img)
        cv2.waitKey(1)


if __name__ == "__main__":
    # line detecting area
    top_left = [0, 0]
    top_right = [640, 0]
    bottom_right = [640, 360]
    bottom_left = [0, 360]
    region = (top_left, top_right, bottom_left, bottom_right)

    buffer_Brake_Yolo = Queue(maxsize=5)
    buffer_Brake_Lidar = Queue(maxsize=5)
    buffer_Slowdown = Queue(maxsize=5)
    buffer_Handle = Queue(maxsize=3)
    buffer_Front_Img = Queue(maxsize=5)

    ########## Lane Curvature buffer ##########
    R_curvature = Queue(maxsize=2)
    R_laneimg = Queue(maxsize=2)
    L_curvature = Queue(maxsize=2)
    L_laneimg = Queue(maxsize=2)
    merged_lane_info = Queue(maxsize=2)

    ############# Lidar Config ######################
    buffer_plot_Lidar = Queue(maxsize=3)
    trigger_distance = 0.15
    
    ########## Lane img Subsrciber Config ##########
    rclpy.init(args=None)
    q_Left = Queue(maxsize=2)
    q_Right = Queue(maxsize=2)

    executor = MultiThreadedExecutor() 
    
    lane_sub1 = Lane_Subscriber(1, 'L_img', q_Left)
    lane_sub2 = Lane_Subscriber(2, 'R_img', q_Right)
    ctrl_pub = CTRL_Publisher(buffer_Handle)
    lidar_node = Lidar_Subscriber(buffer_Brake_Lidar, buffer_plot_Lidar, trigger_distance)
    executor.add_node(lane_sub1)
    executor.add_node(lane_sub2)
    executor.add_node(ctrl_pub)
    executor.add_node(lidar_node)
    print('Done Node Setting')


    # process spawning
    p_LLane = Process(target=lane, args=(region, q_Left, 'L', L_curvature, L_laneimg,))
    p_RLane = Process(target=lane, args=(region, q_Right, 'R', R_curvature, R_laneimg,))
    p_Yolo = Process(target=obj_detection, args=(buffer_Front_Img, buffer_Brake_Yolo, buffer_Slowdown,))
    
    # p_Lidar= Process(target=lidar_info, args=(buffer_Brake_Lidar,))
    p_Ctrl = Process(
        target=merge_lane_info, 
        args=(buffer_Handle,        # Handling data from Lane Detection
            buffer_Slowdown,        # Slowdown data from Yolo
            buffer_Brake_Yolo,      # Braking data from Yolo
            buffer_Brake_Lidar,      # braking data from Lidar
            L_curvature,
            L_laneimg,
            R_curvature,
            R_laneimg,))
    p_LLane.start()
    p_RLane.start()
    p_Yolo.start()
    p_Ctrl.start()    
    
    #p_Lidar.start()

    try:
        while True:
            executor.spin_once()


            if buffer_plot_Lidar.empty():
                continue
            x, y = buffer_plot_Lidar.get()
            plt.figure(1)
            plt.cla()
            plt.ylim(-500,500)
            plt.xlim(-500,500)
            plt.scatter(x, y, c='r', s=13)
            plt.title('Lidar Warning - Hanwool Lee')  
            plt.pause(0.001)

    except Exception as e:
        print("An exception occurred:", type(e).__name__)
        # print('###################################################')
        lane_sub1.destroy_node()
        lane_sub2.destroy_node()
        ctrl_pub.destroy_node()
        lidar_node.destroy_node()
        rclpy.shutdown()  
        # cv2.destroyAllwindows()
        # p_Lane.kill()
        p_LLane.kill() 
        p_RLane.kill()
        p_Yolo.kill()
        p_Ctrl.kill()  
        plt.close()
    
       
