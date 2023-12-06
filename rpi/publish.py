import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge

import cv2
import time
from queue import Queue

import os
import signal
import subprocess

bridge = CvBridge()

class Multi_Publisher(Node):
    def __init__(self, thread_num, topic_name):
        super().__init__('Image_Publisher_'+str(thread_num))
        if topic_name == 'L_img':
            self.cap = cv2.VideoCapture(0, cv2.CAP_V4L)
        else:
            self.cap = cv2.VideoCapture(2, cv2.CAP_V4L)
        # self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        self.topic_name = topic_name
        qos_profile = QoSProfile(
            depth = 10,
            history = HistoryPolicy.KEEP_LAST,
            reliability = ReliabilityPolicy.BEST_EFFORT,
            durability = DurabilityPolicy.VOLATILE
            )

        self.publisher = self.create_publisher(
            CompressedImage,
            topic_name,
            qos_profile)
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.get_logger().info(f'Node{thread_num} created')
        
        time.sleep(1)

    def timer_callback(self):
        _, frame = self.cap.read()
        if self.topic_name == 'L_img':
            frame = cv2.rotate(frame, cv2.ROTATE_180)
        frame = cv2.resize(frame, (640, 360))
        # cv2.imshow(f'{self.topic_name}', frame)
        # cv2.waitKey(1)
        img_msg = bridge.cv2_to_compressed_imgmsg(frame)
        self.publisher.publish(img_msg)

        self.get_logger().info(f'{self.topic_name} ')
        

def main():
    ####### Image Publish #######
    rclpy.init(args=None)
    executor = MultiThreadedExecutor()  # Defualt thread numbers == CPU cores
    sub1 = Multi_Publisher(1, 'L_img')
    sub2 = Multi_Publisher(2, 'R_img')
    executor.add_node(sub1)
    executor.add_node(sub2)

    ####### Lidar Publish #######
    lidar_command = r'ros2 run ydlidar_ros2_driver ydlidar_ros2_driver_node \
                --ros-args --param port:=/dev/ttyUSB0 \
                --ros-args --param frame_id:=laser_frame \
                --ros-args --param baudrate:=128000 \
                --ros-args --param lidar_type:=1 \
                --ros-args --param device_type:=0 \
                --ros-args --param sample_rate:=9 \
                --ros-args --param abnormal_check_count:=4 \
                --ros-args --param resolution_fixed:=true \
                --ros-args --param reversion:=true \
                --ros-args --param inverted:=true \
                --ros-args --param auto_reconnect:=true \
                --ros-args --param isSingleChannel:=false \
                --ros-args --param intensity:=false \
                --ros-args --param support_motor_dtr:=true \
                --ros-args --param angle_max:=180.0 \
                --ros-args --param angle_min:=-180.0 \
                --ros-args --param range_max:=64.0 \
                --ros-args --param range_min:=0.01 \
                --ros-args --param frequency:=10.0 \
                --ros-args --param invalid_range_is_inf:=false'
    sub_p = subprocess.Popen(lidar_command, stdout=subprocess.PIPE, 
                       shell=True, preexec_fn=os.setsid)
    
    print('Ready to Subscribe')

    try:
        while True:
            executor.spin_once()

    except KeyboardInterrupt:
        sub1.destroy_node()
        sub2.destroy_node()
        rclpy.shutdown()
        os.killpg(os.getpgid(sub_p.pid), signal.SIGTERM)
    os.killpg(os.getpgid(sub_p.pid), signal.SIGTERM)

if __name__ == '__main__':
    main()
