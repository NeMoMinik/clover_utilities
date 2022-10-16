import rospy
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from pyzbar import pyzbar
from math import sqrt
from clover import srv
from std_srvs.srv import Trigger
from clover.srv import SetLEDEffect
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Range


rospy.init_node('flight')
qr = ''
bridge = CvBridge()
hsv_min = np.array((0, 0, 0), np.uint8)
hsv_max = np.array((1, 1, 1), np.uint8)
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navi = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)
lent = rospy.ServiceProxy('led/set_effect', SetLEDEffect)

def image_callback(data):
    global qr
    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')  # OpenCV image
    ''' hsv_img = cv.cvtColor(cv_image, cv.COLOR_BGR2HSV)
    thresh = cv.inRange(hsv_img, hsv_min, hsv_max)
    contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)
    cv.drawContours(cv_image, contours, -1, (255,0,0), 3, cv.LINE_AA, hierarchy, 1)
    image_pub.publish(bridge.cv2_to_imgmsg(cv_image, 'bgr8'))'''

    barcodes = pyzbar.decode(cv_image)
    for barcode in barcodes:
        b_data = barcode.data.decode("utf-8")
        b_type = barcode.type
        (x, y, w, h) = barcode.rect
        xc = x + w/2
        yc = y + h/2
        print(b_data)
        qr = b_data

dist = 0
def range_callback(msg):
    global dist
    dist = msg.range

def navi_wait(x=0, y=0, z=0, yaw=float('nan'), speed=0.5, frame_id='', auto_arm=False, tolerance=0.2):
    navi(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)
    print(pose_update)
    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)

def land_wait():
    land()
    while get_telemetry().armed:
        rospy.sleep(0.2)

def lenta(r, g, b, effect='color'):
    global counterr
    lent(r=r, g=g, b=b, effect=effect)

def square(length):
    navi_wait(length, 0, 0, frame_id='navigate_target')
    navi_wait(0, length, 0, frame_id='navigate_target')
    navi_wait(-length, 0, 0, frame_id='navigate_target')
    navi_wait(0, -length, 0, frame_id='navigate_target')

r = rospy.Rate(5)
def pose_update(pose):
    bred = pose.pose.position
    return bred.x, bred.y, bred.z

if __name__ == '__main__':
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pose_update, queue_size=1)
    rospy.Subscriber('/rangefinder/range', Range, range_callback, queue_size=1)
    image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_callback)
    counterr = 0
    navi_wait(0, 0, 1.0, frame_id='body', auto_arm=True)
    navi_wait(1, 0, 1.0)
    while not rospy.is_shutdown() and not qr:
        rospy.sleep(0.2)
    qr2 = qr.split()
    final_color = qr2[0]
    path = list(map(float, qr2[1:]))
    for i in range(0, len(path), 2):
        print(path[i], path[i + 1])
        navi_wait(path[i], path[i + 1], 1.0)
    land_wait()
else:
    image_pub = rospy.Publisher('~debug', Image, queue_size=1)