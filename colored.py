import rospy
from clover import srv
from std_srvs.srv import Trigger
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import math
import numpy as np


rospy.init_node('computer_vision_sample')

bridge = CvBridge()
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)

hsv_min = np.array((63, 54, 92), np.uint8) # 105 67 67   138 255 235
hsv_max = np.array((255, 106, 168), np.uint8)
#hsv_min = np.array((105, 67, 67), np.uint8)
#hsv_max = np.array((138, 255, 235), np.uint8)
center = [0]
detected = [False]
c = [0, 0]
def image_callback(data):
    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')
    hsv_img = cv_image # cv.cvtColor(cv_image, cv.COLOR_BGR2HSV)
    thresh = cv.inRange(hsv_img, hsv_min, hsv_max)
    center[0] = thresh[120][160]
    contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)
    contours = list(filter(lambda x: cv.contourArea(x) > 38, contours))
    if len(contours) != 0:
        cnt = max(contours, key=cv.contourArea)
        epsilon = 0.05 * cv.arcLength(cnt, True)
        cnt = cv.approxPolyDP(cnt, epsilon, True)
        #  cnt = cv.convexHull(cnt) works worse than polydp
        M = cv.moments(cnt)
        try:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            c[:] = [(160 - cx) * 0.03, (120 - cy) * 0.03]
            if c[1] > 0.5:
                c[1] = 0.5
            elif c[1] < -0.5:
                c[1] = -0.5
            if c[0] > 0.5:
                c[0] = 0.5
            elif c[0] < -0.5:
                c[0] = -0.5
            cv.circle(cv_image, (cx, cy), radius=4, color=(0, 255, 0), thickness=-1)
        except:
            pass
        detected[:] = [True]
        print(-c[1], -c[0])
        cv.drawContours(cv_image, cnt, -1, (255,0,0), 3, cv.LINE_AA)
    else:
        c[:] = [0, 0]
    cv.circle(cv_image, (160, 120), radius=4, color=(0, 0, 255), thickness=-1)
    image_pub.publish(bridge.cv2_to_imgmsg(cv_image, 'bgr8'))


def navigate_wait(x=0.0, y=0.0, z=0.0, yaw=float('nan'), speed=0.5, frame_id='', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown() and (not detected[0] or auto_arm):
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
    else:
        print('DETECTED')

def land_wait():
    land()
    while get_telemetry().armed:
        rospy.sleep(0.2)

image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_callback, queue_size=1)
image_pub = rospy.Publisher('~debug', Image, queue_size=1)
'''navigate_wait(0.0, 0.0, 1, frame_id='body', auto_arm=True)
navigate_wait(1.2, 1.2, 1, frame_id='aruco_map')
navigate_wait(1.2, 3, 1, frame_id='aruco_map')
navigate_wait(3.2, 3, 1, frame_id='aruco_map')
navigate_wait(3.2, 1.2, 1, frame_id='aruco_map')
navigate_wait(1.2, 1.2, 1, frame_id='aruco_map')'''
'''if detected[0]:
    while True:
        set_velocity(vx=-c[1], vy=-c[0], vz=0, frame_id='body')'''
rospy.spin()