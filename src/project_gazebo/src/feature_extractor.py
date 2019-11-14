#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

# intialize the node
rospy.init_node("feature_extractor_node")
ini_image_obtained = False
prev_frame = None

# Parameters for ShiTomasi corner detection
feature_params = dict(maxCorners = 1200,
                      qualityLevel = 0.3,
                      minDistance = 7,
                      blockSize = 7)

# Parameters for lucas kanade optical flow
lk_params = dict(winSize = (15,15),
                maxLevel = 2,
                criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

#create some random colors
color = np.random.randint(0, 255, (100, 3))

def harris_corner(image):
    # convert the image to gray scale
    gray_img = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)

    #conversion to float is a prerequisite for the algorithm
    gray_img = np.float32(gray_img)

    # 3 is the size of the neighbourhood considerd, aperture parameter = 3
    # k = 0.04 used to calculate the window score (R)
    corners_img = cv2.cornerHarris(gray_img, 3, 3, 0.04)

    #mark the corners in green
    image[corners_img>0.001*corners_img.max()] = [0,255,0]

    return image

def shi_tomasi(image):
    gray_img =  cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # the parameters in the next line could be tuned if needed
    corner_img = cv2.goodFeaturesToTrack(gray_img,1200,0.01,10)

    for corners in corner_img:
        x, y = corners.ravel()
        cv2.circle(image,(x,y),3,[255,255,0],-1)

    return image

def image_cb(msg):
    global ini_image_obtained, prev_frame
    """
    need to store prev frame
    """
    try:
        cv_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)

    if not ini_image_obtained:
        ini_image_obtained = True
    else:
        cv2.imshow("Prev frame", prev_frame)
        # we will take the first frame and find corners in it
        gray_prev = cv2.cvtColor(prev_frame, cv2.COLOR_BGR2GRAY)
        gray_current = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        pt_prev = cv2.goodFeaturesToTrack(gray_prev, mask = None, **feature_params)

        #create a mask for drawing purpose
        mask = np.zeros_like(prev_frame)
        #lets calculate optical flow now
        pt_curr, st, err = cv2.calcOpticalFlowPyrLK(gray_prev, gray_current, pt_prev, None, **lk_params)
        print(pt_prev[0,:,:], pt_curr[0,:,:])
        #selet good points
        good_new = pt_curr[st==1]
        good_old = pt_prev[st==1]

        #draw the tracks
        for i, (new, old) in enumerate(zip(good_new, good_old)):
            a, b = new.ravel()
            c, d = old.ravel()
            mask = cv2.line(mask, (a,b), (c,d), color[i].tolist(), 2)
            cv_image = cv2.circle(cv_image, (a,b), 5, color[i].tolist(), -1)
        img = cv2.add(cv_image, mask)

        cv2.imshow("current frame", img)
        cv2.waitKey(3)

        #now update the previous frame and previous points
        gray_prev = gray_current.copy()
        pt_prev = good_new.reshape(-1, 1, 2)
    prev_frame =  cv_image

def camera_feed_sub():
    rospy.Subscriber('/pioneer2dx/pioneer2dx/camera/image_raw',Image,image_cb)

if __name__ == "__main__":
    try:
        camera_feed_sub()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
