""" This file contains the classes for cone detection and seeking """
#
# Common functions for cone detection and seeking
#
import cv2, time, rospy
import numpy as np
from threading import Timer

# Needed for publishing the messages
from robo_magellan.msg import pose_data
from robo_magellan.msg import drive_params

def is_cv2():
    # if we are using OpenCV 2, then our cv2.__version__ will start
    # with '2.'
    return check_opencv_version("2.")

def is_cv3():
    # if we are using OpenCV 3.X, then our cv2.__version__ will start
    # with '3.'
    return check_opencv_version("3.")

def check_opencv_version(major):
    # return whether or not the current OpenCV version matches the
    # major version number
    return cv2.__version__.startswith(major)

class ConeFinder:
    """ ConeFinder class """
    codec = 'XVID'

    # On a 640x480 size image, cone area is ~300 sq pixels @25ft
    def __init__(self, min_area=300):
        self.firstTime = True
        self.rgbOut = None
        self.depthOut = None
        self.min_area = min_area

    def _initCapture(self, frame, outFile):
        (h, w) = frame.shape[:2]
        fourcc = cv2.VideoWriter_fourcc(*self.codec)
        capOut = cv2.VideoWriter(outFile, fourcc, 15.0, (w, h), True)
        if not capOut.isOpened():
            capOut = None
            rospy.loginfo("Could not open %s file to write video" % outFile)

        return capOut

    def _getHullDepth(self, hull, depthImg=None):
        """ Returns depth range tuple (min, max) """
        if depthImg is None:
            return (0, 0)

        depthList = []
        # Get bounding box of the hull
        for point in hull:
            depth = depthImg[point[0][1], point[0][0]]
            # R200 provides depth from 50cm to 3.5m typically and each unit is mm
            if depth > 0:
                depthList.append(depth)

        # If we have most of the points with depth, we will assume the rest were error
        if len(depthList) > len(hull)/2:
            depthList.sort()
            return (depthList[0], depthList[-1])

        return (0, 0)

    def _convexHullIsPointingUp(self, hull):
        (centerX, centerY), (w, h), angle = cv2.minAreaRect(hull)

        # Remove any cones with size less than certain threshold
        if h*w < self.min_area:
            #print('Area %d too small' % (h*w))
            return False

        # Our cones are tall, rather than high but angle can affect w and h
        # Angle goes from +90 to -90
        if(angle > 60 or angle < -60):
            aspectRatio = float(h) / w
        elif(angle < 30 and angle > -30):
            aspectRatio = float(w) / h
        else:
            # Very inclined cone, drop them
            #print('Angle (%d) too steep' % angle)
            return False
            
        #if aspectRatio > 0.9:
            #print('Aspect ratio (%f) too low, %d' % (aspectRatio, angle))
            #return False

        listOfPointsAboveCenter = []
        listOfPointsBelowCenter = []

        # step through all points in convex hull
        for point in hull:
            # and add each point to
            # list of points above or below vertical center as applicable
            if point[0][1] < centerY:
                listOfPointsAboveCenter.append(point)
            else:
                listOfPointsBelowCenter.append(point)

        base = 0
        if len(listOfPointsBelowCenter):
            intLeftMostPointBelowCenter = listOfPointsBelowCenter[0][0][0]
            intRightMostPointBelowCenter = listOfPointsBelowCenter[0][0][0]
            # determine left and right most points below center
            for point in listOfPointsBelowCenter:
                if point[0][0] < intLeftMostPointBelowCenter:
                    intLeftMostPointBelowCenter = point[0][0]
                if point[0][0] >= intRightMostPointBelowCenter:
                    intRightMostPointBelowCenter = point[0][0]
            base = intRightMostPointBelowCenter - intLeftMostPointBelowCenter

        top = 0
        if len(listOfPointsAboveCenter):
            intLeftMostPointAboveCenter = listOfPointsAboveCenter[0][0][0]
            intRightMostPointAboveCenter = listOfPointsAboveCenter[0][0][0]
            # determine left and right most points above center
            for point in listOfPointsAboveCenter:
                if point[0][0] < intLeftMostPointAboveCenter:
                    intLeftMostPointAboveCenter = point[0][0]
                if point[0][0] >= intRightMostPointAboveCenter:
                    intRightMostPointAboveCenter = point[0][0]
            top = intRightMostPointAboveCenter - intLeftMostPointAboveCenter
        
        # Top should be smaller than base, ideally only a few pixels
        if (top > base/2):
            #print('top too wide')
            return False

        # Check for slanted cones
        for point in listOfPointsAboveCenter:
            if point[0][0] < intLeftMostPointBelowCenter or \
             point[0][0] > intRightMostPointBelowCenter:
                #print('Slanted cone')
                return False

        # if we get here, shape has passed pointing up check
        return True

    def _process_orange_color(self, img):
        # convert to HSV color space, this will produce better color filtering
        imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Threshold on low range of HSV red
        low_redl = np.array([0, 135, 90])
        low_redh = np.array([19, 255, 255])
        imgThreshLow = cv2.inRange(imgHSV, low_redl, low_redh)

        # threshold on high range of HSV red
        high_redl = np.array([160, 135, 90])
        high_redh = np.array([179, 255, 255])
        imgThreshHigh = cv2.inRange(imgHSV, high_redl, high_redh)

        # combine low range red thresh and high range red thresh
        return cv2.bitwise_or(imgThreshLow, imgThreshHigh)

    def captureFrames(self, cvRGB, cvDepth):
        if self.firstTime:
            # Initialize capture devices
            timestr = time.strftime("%Y%m%d-%H%M")
            rgbOutFile = 'rgb%s.avi' % timestr
            depthOutFile = 'depth%s.avi' % timestr
            if cvRGB is not None:
                self.rgbOut = self._initCapture(cvRGB, rgbOutFile)
            if cvDepth is not None:
                self.depthOut = self._initCapture(cvDepth, depthOutFile)
            self.firstTime = False

        if self.rgbOut is not None and self.rgbOut.isOpened() and cvRGB is not None:
            self.rgbOut.write(cvRGB)
        if self.depthOut is not None and self.depthOut.isOpened() and cvDepth is not None:
            self.depthOut.write(cv2.cvtColor((cvDepth/256).astype('uint8'), cv2.COLOR_GRAY2BGR))

    def release(self):
        if(self.rgbOut is not None):
            self.rgbOut.release()
        if(self.depthOut is not None):
            self.depthOut.release()
        
    def find_cones(self, img, depthImg=None):
        h, w = img.shape[:2]

        image_centerX = w/2
        image_centerY = h  # y goes down from top

        # Process orange color and convert to gray image
        imgThresh = self._process_orange_color(img)
        #imgThresh = cv2.GaussianBlur(imgThresh, (3, 3), 0)
        #imgThresh = cv2.medianBlur(imgThresh, 5)
        imgThresh = cv2.bilateralFilter(imgThresh, 5, 20, 20)

        if is_cv2():
            contours, hierarchy = cv2.findContours(imgThresh, cv2.RETR_EXTERNAL,
                                                   cv2.CHAIN_APPROX_SIMPLE)
        else:
            image, contours, hierarchy = cv2.findContours(imgThresh, cv2.RETR_EXTERNAL,
                                                          cv2.CHAIN_APPROX_SIMPLE)

        listOfHullsAndArea = []
        if len(contours) != 0:
            for cnt in contours:
                epsilon = 0.1 * cv2.arcLength(cnt, True)
                # print'epsilon',epsilon
                contour = cv2.approxPolyDP(cnt, epsilon, True)
                #contour = cv2.approxPolyDP(cnt, 6.7, True)
                # Find convex hulls.
                hull = cv2.convexHull(contour, returnPoints=True)
                # get the depth range for the hull - min and max
                depthRange = self._getHullDepth(hull, depthImg)
                # We need to sort and store the contours by proximity of their centroids
                listOfHullsAndArea.append((hull, cv2.contourArea(hull), depthRange))

        listOfCones = []
        poses = []

        # Sort the list by decreasing area
        listOfHullsAndArea.sort(key=lambda (h, a, d): a, reverse=True)
        for (hull, area, (dMin, dMax)) in listOfHullsAndArea:
            # print 'convexHull',len(temp)
            if len(hull) >= 3 and self._convexHullIsPointingUp(hull):
                listOfCones.append(hull)
                x, y, w, h = cv2.boundingRect(hull)
                pose = pose_data()
                pose.x = x + w/2 - image_centerX
                pose.w = w
                # Height is being measured top of screen to down so we need to invert y
                pose.y = (image_centerY - (y+h))
                pose.h = h
                pose.z = int(dMin)   # But this is the hypotenuse
                pose.d = int(dMax - dMin)
                pose.area = area
                poses.append(pose)

        return (poses, listOfCones)

class ConeSeeker:
    """ ConeSeeker class
        It will produce a steering range of -1. to 1. and
        a throttle value of min_throttle to max_throttle
    """
    # Must be between 0. to 1.
    conf_decay_rate = 0.8
    nItems = 16
    def __init__(self, debug=False):
        self.prev_pos_confs = []
        self.seek_started = False
        self.timer = None
        # Turn right
        self.st_delta = 1.0
        self.cut_throttle_start_time = 0
        self._debug = debug
        if debug:
            self.dpPub = rospy.Publisher('cone_finder/drive_params', drive_params, queue_size=10)

    def _search_timeout(self):
        # Reverse steering values at each timeout
        #self.st_delta = -self.st_delta
        self.timer = None

    def _search_cone(self):
        # start a timer and set a random value for steering_delta
        if self.timer is None:
            # Change st_delta every loiter_period seconds
            self.timer = Timer(rospy.get_param('cone_finder/search_period'), self._search_timeout)
            self.timer.start()
        return (self.st_delta, rospy.get_param('cone_finder/search_throttle'))

    def _update_prev_poses(self):
        new_pos_confs = []
        for (prev_pose, confidence, frame) in self.prev_pos_confs:
            confidence *= self.conf_decay_rate
            frame += 1
            if confidence > 0.05:
                new_pos_confs.append((prev_pose, confidence, frame))

        new_pos_confs.sort(key=lambda (p, c, f): c, reverse=True)
        #print(new_pos_confs[0:2])

        # Keep only top nItems items with high confidence
        self.prev_pos_confs = new_pos_confs[0:self.nItems]

    def _getConfFromOldFrames(self, pose):
        x1 = pose.x - pose.w/2
        x2 = pose.x + pose.w/2
        y1 = pose.y
        y2 = pose.y + pose.h
        conf = 0.0
        matched_poses = []
        for (idx, (prev_pose, prev_conf, frame)) in enumerate(self.prev_pos_confs):
            if frame == 0:
                continue
            old_x1 = prev_pose.x - prev_pose.w/2
            old_x2 = prev_pose.x + prev_pose.w/2
            old_y1 = prev_pose.y
            old_y2 = prev_pose.y + prev_pose.h
            dx = min(x2, old_x2) - max(x1, old_x1)
            dy = min(y2, old_y2) - max(y1, old_y1)
            if (dx >= 0) and (dy >= 0):
                conf += prev_conf * (dx*dy*1.0)/(prev_pose.w*prev_pose.h)
                matched_poses.append(idx)

        return (conf, matched_poses)

    def _get_drive_deltas(self, cone_loc):
        # Steer if not in front, needs more throttle when angle is not sharp
        steering_delta = cone_loc.x/320.0

        #These need to be less than 1
        min_throttle = rospy.get_param('cone_finder/min_throttle')
        max_throttle = rospy.get_param('cone_finder/max_throttle')
        throttle_delta = max_throttle
        # Use real depth when available for throttle
        if cone_loc.z > 0:
            # Real depth is in mm and maximum would probably be less than 5m
            throttle_delta -= (max_throttle - min_throttle) * cone_loc.z/6000.
        else:
            throttle_delta -= (max_throttle - min_throttle) * cone_loc.y/480.
        
        return (steering_delta, throttle_delta)

    def seek_cone(self, poses):
        """ Return steering and throttle adjustments on a 0 to 1 range to drive
            to where the cone with most confidence is
        """
        self._update_prev_poses()
        # Compute confidence for each hull by area and h distance
        confidence = 0.
        if len(poses):
            maxArea = max(pose.area for pose in poses)

            all_matches = []
            new_pos_confs = []
            for pose in poses:
                oldConf, matched_poses = self._getConfFromOldFrames(pose)
                all_matches.extend(matched_poses)

                # Distance is proportional to inverse of pose.y (which is sort of z direction in 3D)
                # Pose.x also scales with distance, more the distance
                pd = 1 + (pose.x/40.0 * pose.y/40.0)**2 + (pose.y/40.0)**2
                # Find this cone among cones from previous frames and use the confidence
                # No need to reduce effect of area as cone gets closer since we want closer cone
                conf = 0.5 * (1/pd + pose.area/maxArea) + oldConf
                new_pos_confs.append((pose, conf, 0))

            # Remove matched cones from the list
            all_matches = list(set(all_matches))
            for idx in sorted(all_matches, reverse=True):
                self.prev_pos_confs.pop(idx)

            # Add old entries to this list and sort by confidence
            new_pos_confs.extend(self.prev_pos_confs)
            new_pos_confs.sort(key=lambda (p, c, f): c, reverse=True)
            
            # There is at least one cone here, confidence is second item
            max_conf = new_pos_confs[0][1]
            if max_conf > 1.0:
                self.prev_pos_confs = []
                # Cap confidence at 1. and add it to 
                for (p, c, f) in new_pos_confs:
                    c /= max_conf
                    self.prev_pos_confs.append((p, c, f))
            else:
                self.prev_pos_confs = new_pos_confs

        # A cone from previous frames might have better confidence
        if len(self.prev_pos_confs):
            (cone_loc, confidence, frame) = self.prev_pos_confs[0]
            self.seek_started = True
            #If confidence falls below threshold, we need to start searching for cone again
            if confidence < rospy.get_param('cone_finder/min_confidence'):
                self.seek_started = False
            if self.seek_started:
                (sd, td) = self._get_drive_deltas(cone_loc)
                # Cut throttle if we get really near
                min_throttle = rospy.get_param('cone_finder/min_throttle')
                if cone_loc.area > rospy.get_param('cone_finder/max_tc_cone_size'):
                    td = 0
                else:
                    if cone_loc.area > rospy.get_param('cone_finder/threshold_tc_cone_size') \
                        and self.cut_throttle_start_time == 0:
                        self.cut_throttle_start_time = rospy.Time.now()
                    # Cut throttle on timeout when we are near
                    if self.cut_throttle_start_time:
                        tc_timeout = rospy.get_param('cone_finder/tc_timeout')
                        tc_max = rospy.get_param('cone_finder/tc_max')
                        if rospy.Time.now() > self.cut_throttle_start_time + rospy.Duration(tc_timeout):
                           td = min_throttle/2
                        if rospy.Time.now() > self.cut_throttle_start_time + rospy.Duration(tc_max):
                            self.cut_throttle_start_time = 0
                if self._debug:
                    dp = drive_params()
                    dp.confidence = int(confidence*100)
                    dp.throttle = int(td*100)
                    dp.steering = int(sd*100)
                    dp.h = cone_loc.h
                    dp.d = cone_loc.d
                    dp.area = cone_loc.area
                    dp.header.stamp = rospy.Time.now()
                    self.dpPub.publish(dp)
                return (cone_loc, confidence, sd, td)

        #This would only happen if the list is empty
        pose = pose_data()
        pose.x = pose.y = pose.z = pose.w = pose.h = pose.d = 0
        pose.area = 0.0
        (sd, td) = self._search_cone()
        if self._debug:
            dp = drive_params()
            dp.confidence = confidence
            dp.throttle = int(td*100)
            dp.steering = 0 if self.seek_started else int(sd*100)
            dp.h = 0
            dp.d = 0
            dp.area = 0
            dp.header.stamp = rospy.Time.now()
            self.dpPub.publish(dp)

        # Saw a cone but confidence is low but not 0
        if self.seek_started and confidence > 0.:
            return (pose, 0.0, 0.0, td)
        
        return (pose, 0.0, sd, td)

