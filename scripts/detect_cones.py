#!/usr/bin/env python

import numpy as np
import cv2, os, argparse, glob
import time, math, sys
import rospy, message_filters, threading
#from operator import itemgetter, attrgetter

# Needed for publishing the messages
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from geometry_msgs.msg import Point
from cone_finder.msg import pose_data
from cone_finder.msg import location_msgs as location_data
from cv_bridge import CvBridge, CvBridgeError

class Args(object):
    use_ros_topic = False
    publish_images = False
    area_threshold = 100
    debug = False
    fromMain = False
    codec = 'XVID'
    image_dir = ''
    video_file = ''
    capture_video = False
    firstTime = True
    rgbOut = None
    depthOut = None
    rgbOutFile = 'rgb.avi'
    depthOutFile = 'depth.avi'
    

args = Args()
pub = rospy.Publisher('cone_finder/locations', location_data, queue_size=10)
colorPub = rospy.Publisher("cone_finder/colorImage", Image, queue_size=10)
depthPub = rospy.Publisher("cone_finder/depthImage", Image, queue_size=10)
colorCIPub = rospy.Publisher("cone_finder/colorCamInfo", CameraInfo, queue_size=2)
depthCIPub = rospy.Publisher("cone_finder/depthCamInfo", CameraInfo, queue_size=2)

def is_cv2():
    # if we are using OpenCV 2, then our cv2.__version__ will start
    # with '2.'
    return check_opencv_version("2.")

def is_cv3():
    # if we are using OpenCV 3.X, then our cv2.__version__ will start
    # with '3.'
    return check_opencv_version("3.")

def check_opencv_version(major, lib=None):
    # if the supplied library is None, import OpenCV
    if lib is None:
        import cv2 as lib

    # return whether or not the current OpenCV version matches the
    # major version number
    return lib.__version__.startswith(major)

#def enhance_depth(cvDepth):
#    D = cvDepth.copy()
    # R200 provides depth from 50cm to 1.4m typically
#    D[cvDepth < 300] = 0
#    D[cvDepth > 8000] = 0
#    D = D * 8
#    return D

def initCapture(frame, outFile):
    (h, w) = frame.shape[:2]
    fourcc = cv2.VideoWriter_fourcc(*args.codec)
    capOut = cv2.VideoWriter(outFile, fourcc, 15.0, (w, h), True)
    if(capOut.isOpened() == False):
        capOut = None
        rospy.loginfo("Could not open %s file to write video" % outFile)    
    
    return capOut            

def captureFrames(cvRGB, cvDepth):
    if(args.firstTime):    
        # Initialize capture devices
        args.rgbOut = initCapture(cvRGB, args.rgbOutFile)
        args.depthOut = initCapture(cvDepth, args.depthOutFile)
        args.firstTime = False

    if(args.rgbOut is not None and args.rgbOut.isOpened() and cvRGB is not None):
        args.rgbOut.write(cvRGB)
    if(args.depthOut is not None and args.depthOut.isOpened() and cvDepth is not None):
        args.depthOut.write(cv2.cvtColor((cvDepth/256).astype('uint8'), cv2.COLOR_GRAY2BGR))

#Returns depth range tuple (min, max, realDepth) - realDepth is a boolean
def getHullDepth(hull, depthImg=None):
    if(depthImg is None):
        return (0, 0, False)

    h = depthImg.shape[:1]
    depthList = []
    # Get bounding box of the hull
    for point in hull:
        depth = depthImg[point.x, point.y]
        # assuming we don't get good values below a threshold (100mm)
        # R200 provides depth from 50cm to 1.4m typically
        if(depth > 300 and depth < 8000):
            depthList.append(depth)

    # If we have most of the points with depth, we will assume the rest were error
    if(len(depthList) > 0.6*len(hull)):
        depthList = sorted(depthList)
        return (depthList[0], depthList[-1], True)

    return (0, 0, False)
    
def convexHullIsPointingUp(hull):
    #x, y, w, h = cv2.boundingRect(hull)
    #centerY = y + h / 2
    
    (centerX, centerY), (w, h), angle = cv2.minAreaRect(hull)
    # Remove any cones with size less than certain threshold
    if(h*w < args.area_threshold):
        return False
    
    # Our cones are tall, rather than high
    aspectRatio = float(w) / h
    if aspectRatio > 0.9:
        return False

    # Very inclined cone, drop them
    if(angle > 30 or angle < -30):
        return False
    
    listOfPointsAboveCenter = []
    listOfPointsBelowCenter = []

    # step through all points in convex hull
    for point in hull:
        # and add each point to
        # list of points above or below vertical center as applicable
        if point[0][1] < centerY:
            listOfPointsAboveCenter.append(point)

        if point[0][1] >= centerY:
            listOfPointsBelowCenter.append(point)

    intLeftMostPointBelowCenter = listOfPointsBelowCenter[0][0][0]
    intRightMostPointBelowCenter = listOfPointsBelowCenter[0][0][0]

    # determine left most point below center
    for point in listOfPointsBelowCenter:
        if point[0][0] < intLeftMostPointBelowCenter:
            intLeftMostPointBelowCenter = point[0][0]

        # determine right most point below center
    for point in listOfPointsBelowCenter:
        if point[0][0] >= intRightMostPointBelowCenter:
            intRightMostPointBelowCenter = point[0][0]

        # step through all points above center
    for point in listOfPointsAboveCenter:
        if point[0][0] < intLeftMostPointBelowCenter or \
         point[0][0] > intRightMostPointBelowCenter:
            return False

    # if we get here, shape has passed pointing up check
    return True

def process_orange_color(img):
    # convert to HSV color space, this will produce better color filtering
    imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Threshold on low range of HSV red
    low_redl = np.array([0, 135, 135])
    low_redh = np.array([19, 255, 255])
    imgThreshLow = cv2.inRange(imgHSV, low_redl, low_redh)

    # threshold on high range of HSV red
    high_redl = np.array([160, 135, 135])
    high_redh = np.array([179, 255, 255])
    imgThreshHigh = cv2.inRange(imgHSV, high_redl, high_redh)

    # combine low range red thresh and high range red thresh
    return cv2.bitwise_or(imgThreshLow, imgThreshHigh)

def find_cones(img, depthImg=None):
    h, w = img.shape[:2]
    
    image_centerX = w/2
    image_centerY = h  # y goes down from top
        
    captureFrames(img, depthImg)
    # Process orange color and convert to gray image
    imgThresh = process_orange_color(img)
    #captureFrames(imgThresh, depthImg)
            
    # clone/copy thresh image before smoothing
    imgThreshSmoothed = imgThresh.copy()
    # open image (erode, then dilate)
    kernel = np.ones((3, 3), np.uint8)
    imgThreshSmoothed = cv2.erode(imgThresh, kernel, iterations=1)
    imgThreshSmoothed = cv2.dilate(imgThreshSmoothed, kernel, iterations=1)
    # Gaussian blur
    imgThreshSmoothed = cv2.GaussianBlur(imgThreshSmoothed, (5, 5), 0)
    #cv2.imshow('imgThreshSmoothed ', imgThreshSmoothed)
    # get Canny edges

    imgCanny = cv2.Canny(imgThreshSmoothed, 160, 80)
    #cv2.imshow('imgCanny ', imgCanny)
    if is_cv2():
        contours, hierarchy = cv2.findContours(imgCanny,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    else:
        image, contours, hierarchy = cv2.findContours(imgCanny,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

    listOfHullsAndArea = []
    if len(contours) != 0:
        for cnt in contours:
            epsilon = 0.1 * cv2.arcLength(cnt, True)
            # print'epsilon',epsilon
            contour = cv2.approxPolyDP(cnt, epsilon, True)
            #contour = cv2.approxPolyDP(cnt, 6.7, True)
            # Find convex hulls.
            hull = cv2.convexHull(contour, returnPoints=True)
            # See how the hull looks as a triangle
            # tri = cv2.minEnclosingTriangle(hull)
            # get the depth for the hull. Is it one value or multiple?
            depthRange = getHullDepth(hull)
            # We need to sort and store the contours by proximity of their centroids
            listOfHullsAndArea.append((hull, cv2.contourArea(hull), depthRange))

    listOfCones = []
    pose = pose_data()
    poses = []

    # Sort the list by decreasing area
    listOfHullsAndArea = sorted(listOfHullsAndArea, key=lambda pair: pair[1], reverse=True)
    for (hull, area, (dMin, dMax, isReal)) in listOfHullsAndArea:
        # print 'convexHull',len(temp)
        if (len(hull) >= 3 and convexHullIsPointingUp(hull)):
            listOfCones.append(hull)
            x, y, w, h = cv2.boundingRect(hull)
            pose.x = x + w/2 - image_centerX
            pose.w = w
            # Height is being measured top of screen to down so we need to invert y
            pose.y = (image_centerY - (y+h))
            pose.h = h
            # Divide depth by 256 since x isn't really in real units
            pose.z = depthRange[0]   # But this is the hypotenuse
            pose.d = depthRange[1] - depthRange[0]
            pose.area = area
            poses.append(pose)

    return (poses, listOfCones)

def find_in_images(loc='../images'):
    # get the files
    files = glob.glob(loc + '/*.jpg')

    rate = rospy.Rate(1) # One image per second
    for file in files:
        if args.debug:
            rospy.logdebug('Processing file %s' % file)
        img = cv2.imread(file, -1)
        poses, listOfCones = find_cones(img)
        cv2.drawContours(img, listOfCones, -1, (0, 255, 0), 3)
        
        if args.debug:
            cv2.imshow('output', img)
            msg_str = 'Found %d Cones' % count
            rospy.logdebug(msg_str)
        rate.sleep()

def find_in_video(fileName):
    if(fileName == None):
      cap = cv2.VideoCapture(0)
    else:
      cap = cv2.VideoCapture(fileName)
      if(cap.isOpened() == False):
        rospy.logerr("Error: Could not open video file " + fileName + ". Using default device.")
        cap = cv2.VideoCapture(0)

    if(cap.isOpened() == False):
      rospy.logerr("Could not open default video device")
      return

    loopCount = 0
    #rate = rospy.Rate(10) # 10 frames per second
    while not rospy.is_shutdown():
        # Capture frame-by-frame
        ret, frame = cap.read()
        loopCount = loopCount + 1

        #TODO: Should we continue or break on bad frame?
        # Display the resulting frame
        if(ret):
            poses, listOfCones = find_cones(frame)
            imghull = frame.copy()
            cv2.drawContours(imghull, listOfCones, -1, (0, 255, 0), 3)
            if args.debug:
                msg_str = 'FS = %.3f' % ((time.clock() - startLoop)/loopCount)
                cv2.putText(imghull, msg_str)
                cv2.imshow('output', imghull)

            if(loopCount == 100):
                msg_str = 'FS = %.3f' % ((time.clock() - startLoop)/loopCount)
                rospy.loginfo(msg_str)
                loopCount = 0
                startLoop = time.clock()

        k = cv2.waitKey(30) & 0xff
        if k == 27:
            break
        #rate.sleep()

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()

class RosColorDepth:
    def __init__(self):
        self.node_name = "RosColorDepth"
        self.bridge = CvBridge()
        self.thread_lock = threading.Lock()
        self.ts = time.clock()
        self.lc = 0
        
        colorCamInfo = message_filters.Subscriber("/camera/color/camera_info", CameraInfo)
        depthCamInfo = message_filters.Subscriber("/camera/depth/camera_info", CameraInfo)
        ts = message_filters.TimeSynchronizer([colorCamInfo, depthCamInfo], 10)
        ts.registerCallback(self.camInfoCallback)
        colorImage = message_filters.Subscriber("/camera/color/image_raw", Image)
        depthImage = message_filters.Subscriber("/camera/depth/image_raw", Image)
        ts = message_filters.TimeSynchronizer([colorImage, depthImage], 10)
        ts.registerCallback(self.imageCallback)
        rospy.loginfo("[%s] Initialized." %(self.node_name))
        rospy.spin()

    def camInfoCallback(self, colorCamInfo, depthCamInfo):
        self.colorCamInfo = colorCamInfo
        self.depthCamInfo = depthCamInfo

    def imageCallback(self, colorImage, depthImage):
        thread = threading.Thread(target=self.processImage, args=(colorImage, depthImage))
        thread.setDaemon(True)
        thread.start()

    def processImage(self, colorImage, depthImage):
        if not self.thread_lock.acquire(False):
            return

        #print(colorImage.encoding, depthImage.encoding)
        cvRGB = self.bridge.imgmsg_to_cv2(colorImage, "bgr8")
        cvDepth = self.bridge.imgmsg_to_cv2(depthImage)

        dh, dw = cvDepth.shape[:2]
        ch, cw = cvRGB.shape[:2]
        if (ch != dh) and (cw != dw): 
            cvRGB = cv2.resize(cvRGB, (dw, dh))

        self.colorCamInfo.width = dw
        self.colorCamInfo.height = dh
        self.depthCamInfo.width = dw
        self.depthCamInfo.height = dh

        try:
            self.lc = self.lc + 1
            poses, listOfCones = find_cones(cvRGB, cvDepth)
            imghull = cvRGB.copy()
            if(len(poses)):
                loc = location_data()
                loc.poses = poses
                loc.header.stamp = rospy.Time.now()
                pub.publish(loc)                
                # Frame first 3 cones - they are sorted by area
                cv2.drawContours(imghull, listOfCones[0:2], -1, (0, 255, 0), 3)
                #captureFrames(cvRGB, cvDepth)
            
            if(args.publish_images):
                msg_str = 'FS = %.3f' % ((time.clock() - self.ts)/self.lc)
                cv2.putText(imghull, msg_str, (0, 460), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (255,0,0), 2, cv2.LINE_AA)
                ts = rospy.Time.now()
                self.colorCamInfo.header.stamp = ts
                self.depthCamInfo.header.stamp = ts
                colorCIPub.publish(self.colorCamInfo)
                depthCIPub.publish(self.depthCamInfo)
                colorMsg = self.bridge.cv2_to_imgmsg(imghull, "bgr8")
                colorMsg.header.stamp = ts
                colorMsg.header.frame_id = colorImage.header.frame_id
                colorPub.publish(colorMsg)
                depthMsg = depthImage
                depthMsg.header.stamp = ts
                #depthMsg.header.frame_id = 'camera_link'
                depthPub.publish(depthMsg)

            if(self.lc == 100):
                msg_str = 'FS = %.3f' % ((time.clock() - self.ts)/self.lc)
                rospy.loginfo(msg_str)
                self.lc = 0
                self.ts = time.clock()
           
        except CvBridgeError as e:
            rospy.logerr(e)
            
        self.thread_lock.release()
    
def find_cones_main():
    rospy.init_node('cone_finder')
    if args.use_ros_topic:
        r = RosColorDepth()
    else:
        if args.image_dir:
            args.debug = True
            find_in_images(args.image_dir)
            # No rospy.spin when working with image dir
        else:
            find_in_video(args.video_file)
            rospy.spin()

if __name__=="__main__":
    parser = argparse.ArgumentParser(description='Find cones in video feed or images')
    parser.add_argument('--use_ros_topic', '-r', action='store_true', help='Use ROS topic')
    parser.add_argument('--image_dir', '-i', help='Find cones in images under specified directory')
    parser.add_argument('--capture_video', '-c', action='store_true', help='Capture videos')
    parser.add_argument('--area_threshold', '-a', type=int, help='Area threshold to use for cones')
    parser.add_argument('--debug', '-d', action='store_true', help='Show debug messages')
    parser.add_argument('--publish_images', '-p', action='store_true', help='Publish marked images')
    parser.add_argument('video_file', nargs='?', help='Find cones in specified video file, use default video device if not specified')
    parser.parse_args(rospy.myargv(sys.argv[1:]), args)
    try:
      find_cones_main()

    except rospy.ROSInterruptException:
      pass
