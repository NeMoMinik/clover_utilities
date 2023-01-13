import rospy
from clover import srv
from std_srvs.srv import Trigger
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import math


rospy.init_node('computer_vision_sample')

bridge = CvBridge()
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)

hsv_min = np.array((0, 0, 0), np.uint8)
hsv_max = np.array((1, 1, 1), np.uint8)


def image_callback(data):
    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')  # OpenCV image
    hsv_img = cv.cvtColor(cv_image, cv.COLOR_BGR2HSV)
    thresh = cv.inRange(hsv_img, hsv_min, hsv_max)
    contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)
    cnt = contours[0]
    M = cv.moments(cnt)
    cv.drawContours(cv_image, contours, -1, (255,0,0), 3, cv.LINE_AA, hierarchy, 1)
    image_pub.publish(bridge.cv2_to_imgmsg(cv_image, 'bgr8'))

def navigate_wait(x=0, y=0, z=0, yaw=float('nan'), speed=0.5, frame_id='', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break

def land_wait():
    land()
    while get_telemetry().armed:
        rospy.sleep(0.2)

image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_callback)
image_pub = rospy.Publisher('~debug', Image, queue_size=1)
navigate_wait(0, 0, 2, frame_id='body', auto_arm=True)
navigate_wait(1.2, 1.2, 2, frame_id='aruco_map')
navigate_wait(1.2, 3, 2, frame_id='aruco_map')
navigate_wait(3.2, 3, 2, frame_id='aruco_map')
navigate_wait(3.2, 1.2, 2, frame_id='aruco_map')
navigate_wait(1.2, 1.2, 2, frame_id='aruco_map')
land_wait()

rospy.spin()