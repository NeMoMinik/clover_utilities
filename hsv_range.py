import numpy as np
import cv2
def nothing(args):pass

cv2.namedWindow("setup")
cv2.createTrackbar("v1", "setup", 0, 255, nothing)
cv2.createTrackbar("s1", "setup", 0, 255, nothing)
cv2.createTrackbar("h1", "setup", 0, 180, nothing)
cv2.createTrackbar("v2", "setup", 255, 255, nothing)
cv2.createTrackbar("s2", "setup", 255, 255, nothing)
cv2.createTrackbar("h2", "setup", 180, 180, nothing)

fn = "/home/clover/Desktop/snapshot.jfif" 
img = cv2.imread(fn)
img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

while True:
    h1 = cv2.getTrackbarPos('h1', 'setup')
    s1 = cv2.getTrackbarPos('s1', 'setup')
    v1 = cv2.getTrackbarPos('v1', 'setup')
    h2 = cv2.getTrackbarPos('h2', 'setup')
    s2 = cv2.getTrackbarPos('s2', 'setup')
    v2 = cv2.getTrackbarPos('v2', 'setup')
    min_p = (h1, s1, v1)
    max_p = (h2, s2, v2)
    img_g = cv2.inRange(img, min_p, max_p)

    cv2.imshow('img', img_g)
    
    if cv2.waitKey(33) & 0xFF == ord('q'):
         break

cv2.destroyAllWindows()