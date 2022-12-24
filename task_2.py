import rospy
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from pyzbar import pyzbar


rospy.init_node('computer_vision_sample')

bridge = CvBridge()
hsv_min = np.array((0, 0, 0), np.uint8)
hsv_max = np.array((1, 1, 1), np.uint8)


def image_callback(data):
    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')  # OpenCV image
    hsv_img = cv.cvtColor(cv_image, cv.COLOR_BGR2HSV)
    thresh = cv.inRange(hsv_img, hsv_min, hsv_max)
    contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)
    cv.drawContours(cv_image, contours, -1, (255,0,0), 3, cv.LINE_AA, hierarchy, 1)
    image_pub.publish(bridge.cv2_to_imgmsg(cv_image, 'bgr8'))

    barcodes = pyzbar.decode(cv_image)
    for barcode in barcodes:
        b_data = barcode.data.decode("utf-8")
        b_type = barcode.type
        (x, y, w, h) = barcode.rect
        xc = x + w/2
        yc = y + h/2
        b = []
        b.append(b_data[b_data.index('=') + 1:b_data.index(' ')])
        b_data = b_data[b_data.index(' ') + 1:]
        b.append(b_data[b_data.index('=') + 1:])
        print(b)

image_sub = rospy.Subscriber('main_camera/image_raw', Image, image_callback)
image_pub = rospy.Publisher('~debug', Image, queue_size=1)
rospy.spin()