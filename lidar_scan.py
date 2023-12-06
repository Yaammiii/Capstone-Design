import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
import math
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
import time
from multiprocessing import Process, Queue
from rclpy.executors import MultiThreadedExecutor



qos_profile = QoSProfile(
            depth = 5,
            history = HistoryPolicy.KEEP_LAST,
            reliability = ReliabilityPolicy.BEST_EFFORT,
            durability = DurabilityPolicy.VOLATILE)
class Lidar_Subscriber(Node):
    def __init__(self, buffer_plot_Lidar, trigger_distance):
        super().__init__('Lidar_Subscriber')
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
        x, y, is_true = draw(msg, trigger_distance=self.trigger_distance)
        self.buffer_plot_Lidar.put([x,y])


# plt.ion()
# fig, ax = plt.subplots()

def draw(msg, trigger_distance=0.1):
    length = len(msg.ranges)
    
    # search 120 degree from searching point
    s_point = int(length/10*3.3)
    scan_range = int(length/3)
    search_angle = list(np.linspace(s_point, s_point+scan_range, scan_range+1, dtype=np.uint16))

    dist_info = np.array([msg.ranges[i] for i in search_angle if msg.ranges[i]<1])
    dist_info[dist_info==0] = float('inf')
    x = [msg.ranges[i]*math.cos((2*(i/length)*math.pi))*1000 for i in search_angle if msg.ranges[i]<1]
    y = [msg.ranges[i]*math.sin((2*(i/length)*math.pi))*1000 for i in search_angle if msg.ranges[i]<1]
    
    # ax.clear()
    # ax.plot(x, y, 'ro', markersize=4)
    # plt.xlim(-3000, 3000)
    # plt.ylim(-3000, 3000)
    # plt.show()
    # plt.pause(0.0001)


    n_triggers = sum(dist_info < 0.15)

    if n_triggers/len(dist_info) > trigger_distance:
        print('Warning! Object Too Close '+ str(n_triggers/len(dist_info)))
        return x, y, True
    return x, y, False
    

def callback(msg):
    draw(msg)

def main():
    rclpy.init()
    # node = rclpy.create_node('LiDAR_Sub')
    # node.create_subscription(LaserScan, '/scan', callback, qos_profile_sensor_data)
    buffer_plot_Lidar = Queue(maxsize=3)
    trigger_distance = 0.15
    executor = MultiThreadedExecutor() 

    lidar_node = Lidar_Subscriber(buffer_plot_Lidar, trigger_distance)

    # Wait for messages to arrive
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

        lidar_node.destroy_node()
        rclpy.shutdown()  
        # cv2.destroyAllwindows()
        # p_Lane.kill()

        plt.close()

if __name__ == '__main__':
    main()
