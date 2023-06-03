import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge
from time import sleep

class RSPi_Publisher(Node):
    def __init__(self, 
                 node_name = 'RSPiPub_Node',
                 topic_name = 'Lane_Img'
                 ):
        super().__init__(node_name)
        self._publisher = self.create_publisher(Image, topic_name, qos_profile_sensor_data)
        self.count = 0  # to count how many messages you got
        self.bridge = CvBridge()

    def send(self, img):
        msg = self.bridge.cv2_to_imgmsg(img, header=Header())
        msg.header.frame_id = 'Integrated Image'
        self._publisher.publish(msg)
        self.get_logger().info('Published ' + str(self.count)+'th Message')
        self.count += 1

def main():
    rclpy.init(args=None)
    Publisher1 = RSPi_Publisher(node_name = 'RSPiPub_Node',
                                topic_name = 'Lane_Img'
                                )

    capture1 = cv2.VideoCapture(0)
    capture1.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    capture1.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    capture2 = cv2.VideoCapture(2)
    capture2.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    capture2.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    capture3 = cv2.VideoCapture(4)
    capture3.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    capture3.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

    while True:
        try:
            ret1, frame1 = capture1.read()
            ret2, frame2 = capture2.read()
            ret3, frame3 = capture3.read()

            if not ret1:
                print('cam1 not opened')
                break
            if not ret2:
                print('cam2 not opened')
                break
            if not ret3:
                print('cam3 not opened')
                break

            merged_img = cv2.hconcat([frame1, frame2, frame3])
            cv2.imshow('merged', merged_img)
            cv2.waitKey(1)
            Publisher1.send(merged_img)
            sleep(0.05)
        except KeyboardInterrupt as ke:
            print(ke)
            break
            
        except:
            pass


if __name__ == '__main__':
    main()