#!/usr/bin/env python

""" This file contains code for detecting cones """
import numpy as np, cv2, argparse, time, sys
import rospy, threading

# Needed for publishing the messages
from sensor_msgs.msg import Image, CameraInfo
from robo_magellan.msg import pose_data
from robo_magellan.msg import location_msgs as location_data
from cv_bridge import CvBridge, CvBridgeError
from cone_finder import ConeFinder, ConeSeeker

class RosColorDepth:
    node_name = "cone_finder"
    bridge = CvBridge()
    thread_lock = threading.Lock()
    ts = time.clock()
    cf = None
    cs = None
    lc = 0
    pub = rospy.Publisher('cone_finder/locations', location_data, queue_size=10)
    colorPub = rospy.Publisher("cone_finder/colorImage", Image, queue_size=10)
    depthPub = rospy.Publisher("cone_finder/depthImage", Image, queue_size=10)

    def __init__(self):
        rospy.init_node('cone_finder')
        self.started = True
        self.colorCamInfo = CameraInfo()
        self.depthCamInfo = CameraInfo()

        minArea = rospy.get_param("~minConeArea", 300)
        self.cf = ConeFinder(minArea)
        self.cs = ConeSeeker()

        rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.colorCamInfoCallback)
        rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.depthCamInfoCallback)
        rospy.Subscriber("/camera/color/image_raw", Image, self.imageCallback)
        self.depthImage = None
        rospy.Subscriber("/camera/depth/image_raw", Image, self.depthCallback)

        self.capture_video = rospy.get_param("~captureVideo", False)
        self.publish_images = rospy.get_param("~publishImages", False)

        self.thresholdAlgorithm = rospy.get_param('~thresholdAlgorithm', 'bin')
        self.cf.setThresholdAlgorithm(self.thresholdAlgorithm)

        self.contourFilterAlgorithm = rospy.get_param('~contourFilterAlgorithm', 'convexNull')
        self.cf.setContourFilterAlgorithm(self.contourFilterAlgorithm)

        if rospy.has_param('~binConfig'):
            binConfig = rospy.get_param('~binConfig')
            rospy.loginfo('Using bin configuration from %s' % binConfig)
            self.cf.setBinConfiguration(binConfig)

        self.cs.setIgnorePriorDetections(rospy.get_param('~ignorePriorDetections', True))

        rospy.loginfo('Threshold algorithm %s' % self.thresholdAlgorithm)
        rospy.loginfo("[%s] Initialized." %(self.node_name))
        rospy.spin()

    def colorCamInfoCallback(self, colorCamInfo):
        self.colorCamInfo = colorCamInfo

    def depthCamInfoCallback(self, depthCamInfo):
        self.depthCamInfo = depthCamInfo

    def imageCallback(self, colorImage):
        thread = threading.Thread(target=self.processImage, args=(colorImage, self.depthImage))
        thread.setDaemon(True)
        thread.start()

    def depthCallback(self, depthImage):
        self.depthImage = depthImage

    def start(self):
        self.started = True

    def pause(self):
        self.started = False

    def stop(self):
        self.started = False

    def markVideo(self, imghull, poses):
        (cl, conf, sadj, tadj) = self.cs.seek_cone(poses)
        if conf > 0.1:
            #frame could be non zero
            (ih, iw) = imghull.shape[:2]
            pt1 = (iw/2 + cl.x - cl.w/2, ih - cl.y - cl.h)
            pt2 = (iw/2 + cl.x + cl.w/2, ih - cl.y)
            cv2.rectangle(imghull, pt1, pt2, (0, 0, 255), 3)
            msg_str = '%.2f' % conf
            cv2.putText(imghull, msg_str, pt1, cv2.FONT_HERSHEY_SIMPLEX,
                        1, (0, 0, 255), 2, cv2.LINE_AA)

        msg_str = 'FS = %.3f' % ((time.clock() - self.ts)/self.lc)
        cv2.putText(imghull, msg_str, (10, 460), cv2.FONT_HERSHEY_SIMPLEX,
                    1, (255, 0, 0), 2, cv2.LINE_AA)
        cv2.putText(imghull, 'alg:' + self.thresholdAlgorithm, (300, 460), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2, cv2.LINE_AA)

    def publishImages(self, imghull, colorImage, depthImage):
        ts = rospy.Time.now()

        # Convert from the OpenCV images to ROS image messages and
        # publish the marked-up images.
        # (Why are we publishing the depth image when we haven't
        # added any information to it?)
        colorMsg = self.bridge.cv2_to_imgmsg(imghull, "bgr8")
        colorMsg.header.stamp = ts
        colorMsg.header.frame_id = colorImage.header.frame_id
        self.colorPub.publish(colorMsg)
        if depthImage is not None:
            depthMsg = depthImage
            depthMsg.header.stamp = ts
            #depthMsg.header.frame_id = 'camera_link'
            self.depthPub.publish(depthMsg)

    def processImage(self, colorImage, depthImage):
        # Skip this image if we're already processing an image.
        # That is, if we cannot acquire the thread lock with no
        # delay, then another thread has the lock and is processing
        # an image. This reduces our effective frame rate to what
        # we can process.
        if not self.thread_lock.acquire(False):
            return

        if not self.started:
            return

        # Convert from a ROS image message to an OpenCV image.
        cvRGB = self.bridge.imgmsg_to_cv2(colorImage, "bgr8")
        ch, cw = cvRGB.shape[:2]

        # Convert the depth image, if we have one.
        if depthImage is not None:
            cvDepth = self.bridge.imgmsg_to_cv2(depthImage)
            dh, dw = cvDepth.shape[:2]
        else:
            cvDepth = None
            dh, dw = ch, cw

        # If the depth image is a different size then the color image,
        # resize the color image to match. (This assumes that the
        # field of view of the two cameras is the same. Is this true?
        # It seems better to convert the coordinates of the color
        # image to world coordinates and then to depth image coordinates
        # to find what portion of the depth image can be correlated to
        # the RGB image, then either padding the depth image with zeros,
        # if smaller than the RGB, or cropping to match the RGB field
        # of view, if larger than the RGB.)
        if (ch != dh) and (cw != dw):
            cvRGB = cv2.resize(cvRGB, (dw, dh))
            ch, cw = dh, dw

        self.colorCamInfo.width = cw
        self.colorCamInfo.height = ch
        if self.depthCamInfo is not None:
            self.depthCamInfo.width = dw
            self.depthCamInfo.height = dh

        try:
            self.lc = self.lc + 1
            poses, listOfCones = self.cf.find_cones(cvRGB, cvDepth)
            # Use this function to capture video - here unmodified video
            if self.capture_video:
                self.cf.captureFrames(cvRGB, cvDepth)

            imghull = cvRGB.copy()
            loc = location_data()
            loc.poses = poses
            loc.header.stamp = rospy.Time.now()
            self.pub.publish(loc)

            if self.publish_images:
                if len(poses):
                    # Frame big 3 cones - they are sorted by area
                    cv2.drawContours(imghull, listOfCones[0:2], -1, (0, 255, 0), 3)
                self.markVideo(imghull, poses)
                self.publishImages(imghull, colorImage, depthImage)

            if self.lc % 100 == 0:
                msg_str = 'Frames: {0:d} time per frame: {1:.3f}s'.format(self.lc, (time.clock() - self.ts)/self.lc)
                rospy.loginfo(msg_str)
                self.lc = 0
                self.ts = time.clock()

        except CvBridgeError as e:
            rospy.logerr(e)

        self.thread_lock.release()

def find_cones_main():
    r = RosColorDepth()

if __name__ == "__main__":
    try:
        find_cones_main()

    except rospy.ROSInterruptException:
        pass
