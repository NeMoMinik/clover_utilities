from mavros_msgs.msg import RCIn
import math
import rospy
from clover import srv
from std_srvs.srv import Trigger

rospy.init_node('flight')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)


def navigate_wait(x=0, y=0, z=0, yaw=float('nan'), speed=0.5, frame_id='', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)


def land_wait():
    land()
    while get_telemetry().armed:
        rospy.sleep(0.2)

f = [True]
def rc_callback(data):
    if data.channels[7] > 1900:
        f[:] = [False]
        
navigate_wait(z=1.5, frame_id='body', auto_arm=True)
navigate_wait(x=0.6, y=0.6, z=1.5, frame_id='aruco_map')
navigate_wait(x=1.8, y=1.8, z=1.5, frame_id='aruco_map')
navigate_wait(x=0.6, y=1.8, z=1.5, frame_id='aruco_map')

rospy.Subscriber('mavros/rc/in', RCIn, rc_callback)
while f[0]:
    pass
navigate_wait(x=0.6, y=0.6, z=1.5, frame_id='aruco_map')

rospy.spin()