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

class Args(object):
    publish_images = False
    capture_video = False

args = Args()

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
    colorCIPub = rospy.Publisher("cone_finder/colorCamInfo", CameraInfo, queue_size=2)
    depthCIPub = rospy.Publisher("cone_finder/depthCamInfo", CameraInfo, queue_size=2)

    def __init__(self):
        rospy.init_node('cone_finder')
        self.started = True
        self.colorCamInfo = None
        self.depthCamInfo = None

        minArea = rospy.get_param("~minConeArea", 300)
        self.cf = ConeFinder(minArea)

        rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.colorCamInfoCallback)
        rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.depthCamInfoCallback)
        rospy.Subscriber("/camera/color/image_raw", Image, self.imageCallback)
        self.depthImage = None
        rospy.Subscriber("/camera/depth/image_raw", Image, self.depthCallback)

        self.thresholdAlgorithm = rospy.get_param('~thresholdAlgorithm', 'bin')
        self.cf.setThresholdAlgorithm(self.thresholdAlgorithm)

        self.contourFilterAlgorithm = rospy.get_param('~contourFilterAlgorithm', 'convexNull')
        self.cf.setContourFilterAlgorithm(self.contourFilterAlgorithm)

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
        if self.cs is None:
            # ConeSeeker only used to mark cone confidence, default throttle is good
            self.cs = ConeSeeker()

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
        self.colorCamInfo.header.stamp = ts
        self.colorCIPub.publish(self.colorCamInfo)
        if self.depthCamInfo is not None:
            self.depthCamInfo.header.stamp = ts
            self.depthCIPub.publish(self.depthCamInfo)
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
        if not self.thread_lock.acquire(False):
            return

        if not self.started:
            return

        #print(colorImage.encoding, depthImage.encoding)
        cvRGB = self.bridge.imgmsg_to_cv2(colorImage, "bgr8")
        ch, cw = cvRGB.shape[:2]

        if depthImage is not None:
            cvDepth = self.bridge.imgmsg_to_cv2(depthImage)
            dh, dw = cvDepth.shape[:2]
            ch, cw = dh, dw
        else:
            cvDepth = None
            dh, dw = ch, cw

        if (ch != dh) and (cw != dw):
            cvRGB = cv2.resize(cvRGB, (dw, dh))

        self.colorCamInfo.width = cw
        self.colorCamInfo.height = ch
        if self.depthCamInfo is not None:
            self.depthCamInfo.width = dw
            self.depthCamInfo.height = dh

        try:
            self.lc = self.lc + 1
            poses, listOfCones = self.cf.find_cones(cvRGB, cvDepth)
            # Use this function to capture video - here unmodified video
            if args.capture_video:
                self.cf.captureFrames(cvRGB, cvDepth)

            imghull = cvRGB.copy()
            loc = location_data()
            loc.poses = poses
            loc.header.stamp = rospy.Time.now()
            self.pub.publish(loc)
            if len(poses):
                # Frame big 3 cones - they are sorted by area
                cv2.drawContours(imghull, listOfCones[0:2], -1, (0, 255, 0), 3)

            if args.publish_images:
                self.markVideo(imghull, poses)
                self.publishImages(imghull, colorImage, depthImage)

            if self.lc == 100:
                msg_str = 'FS = %.3f' % ((time.clock() - self.ts)/self.lc)
                rospy.loginfo(msg_str)
                self.lc = 0
                self.ts = time.clock()

        except CvBridgeError as e:
            rospy.logerr(e)

        self.thread_lock.release()

def find_cones_main():
    r = RosColorDepth()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Find cones in published video feeds')
    parser.add_argument('--capture_video', '-c', action='store_true', help='Capture videos')
    parser.add_argument('--publish_images', '-p', action='store_true', help='Publish marked images')
    parser.parse_args(rospy.myargv(sys.argv[1:]), args)
    try:
        find_cones_main()

    except rospy.ROSInterruptException:
        pass
