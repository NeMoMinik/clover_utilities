import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from time import time

rospy.init_node('computer_vision_sample')
bridge = CvBridge()
out = cv2.VideoWriter('outpy_test.avi', cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 10,(320, 240 ))

def image_callback(data):
    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')
    out.write(cv_image)
    cv2.imwrite('images/{}.jpg'.format(time()), cv_image)

image_sub = rospy.Subscriber('main_camera/image_raw', Image, image_callback)

while not rospy.is_shutdown():
    pass

out.release()