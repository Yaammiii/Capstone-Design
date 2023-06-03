import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
import threading
import math
import os

def run_lidar_scan():
    os.system('ros2 launch ydlidar_x4_example ydlidar_x4_example.launch.py')

def draw(msg):
    length = len(msg.ranges)
    x = [-msg.ranges[i]*math.cos((2*(i/length)*math.pi))*1000 for i in range(length)]
    y = [msg.ranges[i]*math.sin((2*(i/length)*math.pi))*1000 for i in range(length)]

    #while True:
    plt.figure(1)
    plt.cla()
    plt.ylim(-3000,3000)
    plt.xlim(-3000,3000)
    plt.scatter(x, y, c='r', s=13)
    #print(q)
    #plt.scatter(q, r, c='b', s=13)
    plt.pause(0.001)
    #plt.close("all")

def data_raw(data):
    for angle in range(0,360):
            if(data[angle]>1000):
                x[angle] = data[angle] * math.cos(math.radians(angle))
                y[angle] = data[angle] * math.sin(math.radians(angle))

def callback(msg):
    # This function will be called when a message is received
    #print('Received message:')
    
    # choose one and uncomment to plot
    #data_filter(msg.data)
    draw(msg)

def main():
    rclpy.init()
    node = rclpy.create_node('LiDAR_Sub')
    node.create_subscription(LaserScan, '/scan', callback, qos_profile_sensor_data)

    threading.Thread(target=run_lidar_scan).start()   # run plotting on the background

    # Wait for messages to arrive
    while rclpy.ok():
        rclpy.spin_once(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
