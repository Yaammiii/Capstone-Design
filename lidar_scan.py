import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
import math
import numpy as np

def draw(msg):
    length = len(msg.ranges)
    
    # search 120 degree from searching point
    s_point = int(length/10*3.3)
    scan_range = int(length/3)
    search_angle = list(np.linspace(s_point, s_point+scan_range, scan_range+1, dtype=np.uint16))

    dist_info = np.array([msg.ranges[i] for i in search_angle if msg.ranges[i]<1])
    dist_info[dist_info==0] = float('inf')
    x = [msg.ranges[i]*math.cos((2*(i/length)*math.pi))*1000 for i in search_angle if msg.ranges[i]<1]
    y = [msg.ranges[i]*math.sin((2*(i/length)*math.pi))*1000 for i in search_angle if msg.ranges[i]<1]

    # rate of too close points that trigger alert
    trigger_rate = 0.1

    n_triggers = sum(dist_info < 0.15)

    if n_triggers/len(dist_info) > trigger_rate:
        print('Warning! Object Too Close '+ str(n_triggers/len(dist_info)))
        return x, y, True
    return x, y, False
    

def callback(msg):
    draw(msg)

def main():
    rclpy.init()
    node = rclpy.create_node('LiDAR_Sub')
    node.create_subscription(LaserScan, '/scan', callback, qos_profile_sensor_data)

    # Wait for messages to arrive
    while rclpy.ok():
        rclpy.spin_once(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
