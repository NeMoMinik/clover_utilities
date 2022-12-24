import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from time import time
from clover import srv
from std_srvs.srv import Trigger
import math

bridge = CvBridge()
out = cv2.VideoWriter('outpy_test.avi', cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 10,(320, 240 ))

rospy.init_node('flight')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)


def image_callback(data):
    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')
    out.write(cv_image)
    cv2.imwrite('images/{}.jpg'.format(time()), cv_image)

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


image_sub = rospy.Subscriber('main_camera/image_raw', Image, image_callback)
navigate_wait(0, 0, 1, auto_arm=True)
for i in range(-5, 5):
    navigate_wait(i, abs(25 - i ** 2) ** 0.5 + 5, 2, speed=1)
    print(i, abs(25 - i ** 2) ** 0.5)

out.release()