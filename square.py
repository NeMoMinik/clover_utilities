from math import sqrt
import rospy
from clover import srv
from std_srvs.srv import Trigger
from clover.srv import SetLEDEffect
from geometry_msgs.msg import PoseStamped

rospy.init_node('flight')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navi = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)
lent = rospy.ServiceProxy('led/set_effect', SetLEDEffect)

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
    print('Position', counterr)
    counterr += 1

r = rospy.Rate(5)
def pose_update(pose):
    bred = pose.pose.position
    print(bred.x, bred.y, bred.z)
    r.sleep()

if __name__ == '__main__':
    
    coords = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pose_update, queue_size=1)
    counterr = 0
    navi(z=1, frame_id='body', auto_arm=True)
    rospy.sleep(2)
    lenta(r=0, g=255, b=0)
    navi_wait(x=0, y=0, z=1, speed=1, frame_id='aruco_map')
    lenta(r=255, g=255, b=255)
    navi_wait(x=0, y=4, z=1, speed=1, frame_id='aruco_map')
    lenta(r=0, g=255, b=255)
    navi_wait(x=1.5, y=2, z=1, speed=1, frame_id='aruco_map')
    lenta(r=0, g=0, b=255)
    navi_wait(x=3, y=4, z=1, speed=1, frame_id='aruco_map')
    lenta(r=255, g=0, b=255)
    navi_wait(x=3, y=0, z=1, speed=1, frame_id='aruco_map')
    lenta(r=255, g=90, b=0)
    navi_wait(x=0, y=0, z=1, speed=1, frame_id='aruco_map')
    lenta(r=255, g=255, b=0)
    navi_wait(x=1, y=3, z=1, speed=1, frame_id='aruco_map')

    lenta(r=0, g=255, b=0)
    lenta(effect='blink', r=0, g=255, b=0)
    rospy.sleep(3)

    land_wait()