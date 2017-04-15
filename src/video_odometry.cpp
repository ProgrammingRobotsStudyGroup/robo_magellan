// video_odometry.cpp - ROS node to publish odometry from video images.

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

#include <iostream>
#include <cmath>

int frameRate;

int horizonWidth, horizonHeight;
int odomWidth, odomHeight, odomMargin;

int horizonFeatures;
double horizonQuality;
int horizonMinDistance;
int horizonWindowSize;
  
int odomFeatures;
double odomQuality;
int odomMinDistance;
int odomWindowSize;

double headingScaling;
double odomScaling;

bool publishFlow;
  
std::string odomFrame;
std::string childFrame;

ros::Time frameTime;

long frameCount;

cv::Mat horizonMask;
cv::Mat odomMask;

cv::Size horizonWindow;
cv::Size odomWindow;

cv::Ptr<cv::Feature2D> horizonDetector;
cv::Ptr<cv::Feature2D> odomDetector;

cv::Mat lastGray;
std::vector<cv::Point2f> lastHorizonPoints;
std::vector<cv::Point2f> lastOdomPoints;

ros::Publisher odomPub;
ros::Publisher imagePub;

const double PI = 3.1415926536;

double x = 0;
double y = 0;
double theta = 0;
int direction = 0;
double lastDv = 0;

double normalizeAngle(double phi) {
  while (phi < 0) {
    phi += 2*PI;
  }

  while (phi >= 2*PI) {
    phi -= 2*PI;
  }

  return phi;
}

double normalizeDeltaAngle(double phi) {
  while (phi <= -PI) {
    phi += 2*PI;
  }

  while (x > PI) {
    phi -= 2*PI;
  }

  return phi;
}

void publishOdometry(double dcx, double dcy, double dvx, double dvy) {
  bool hasVelocity = !isnan(dvx);

  double newTheta = normalizeAngle(theta + headingScaling*dcx);
  double dTheta = normalizeDeltaAngle(newTheta - theta);

  double adjDvy = dvy - dcy;
  double dv = lastDv;
  if (hasVelocity) {
    if (abs(dvx) < 0.01) {
      dv = 0;
    } else {
      dv = sqrt(dvx*dvx + adjDvy*adjDvy);
    }
  }

  if (dvx != 0) {
    direction = atan2(dvx, adjDvy);
  }

  int dirSign = (fabs(direction) <= PI/2 ? 1 : -1);

  double ds = dv * dirSign * odomScaling;
  double speed = ds * frameRate;

  double newX, newY;
  if (dTheta == 0) {
    newX = x + ds*cos(theta);
    newY = y + ds*sin(theta);
  } else {
    double R = ds / dTheta;
    double iccX = x - R*sin(theta);
    double iccY = y + R*cos(theta);

    newX = (x - iccX)*cos(dTheta) - (y - iccY)*sin(dTheta) + iccX;
    newY = (x - iccX)*sin(dTheta) + (y - iccY)*cos(dTheta) + iccY;
  }

  x = newX;
  y = newY;
  theta = newTheta;
  double dThetaDt = dTheta * frameRate;

  geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(theta);

  nav_msgs::Odometry odom;
  odom.header.stamp = frameTime;
  odom.header.frame_id = odomFrame.c_str();

  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0;
  odom.pose.pose.orientation = q;

  odom.child_frame_id = childFrame.c_str();
  odom.twist.twist.linear.x = speed * cos(theta);
  odom.twist.twist.linear.y = speed * sin(theta);
  odom.twist.twist.linear.z = dThetaDt;

  odomPub.publish(odom);
}

void makeMasks(const cv::Mat &frame) {
  cv::Scalar ones(255, 255, 255);

  cv::Point2f horizonNW(frame.cols/2 - horizonWidth/2, 0);
  cv::Point2f horizonSE(frame.cols/2 + horizonWidth/2, horizonHeight);
  horizonMask = cv::Mat::zeros(frame.rows, frame.cols, CV_8UC1);
  cv::rectangle(horizonMask, horizonNW, horizonSE, ones, CV_FILLED);

  cv::Point2f odomNW(frame.cols/2 - odomWidth/2,
		     frame.rows - odomMargin - odomHeight);
  cv::Point2f odomSE(frame.cols/2 + odomWidth/2,
		     frame.rows - odomMargin);
  odomMask = cv::Mat::zeros(frame.rows, frame.cols, CV_8UC1);
  cv::rectangle(odomMask, odomNW, odomSE, ones, CV_FILLED);

  horizonWindow = cv::Size(horizonWindowSize, horizonWindowSize);
  odomWindow = cv::Size(odomWindowSize, odomWindowSize);
}

void findFlow(cv::Mat &frame, const cv::Mat &gray,
	      std::vector<cv::Point2f> horizonPoints,
	      std::vector<cv::Point2f> odomPoints) {

  std::vector<uchar> status;
  std::vector<float> error;

  cv::calcOpticalFlowPyrLK(lastGray, gray, lastHorizonPoints, horizonPoints,
			   status, error, horizonWindow);
  cv::Mat horizonInliers;
  cv::Mat horizonH = cv::findHomography(lastHorizonPoints, horizonPoints,
					cv::RANSAC, 2.5F, horizonInliers);

  if (horizonH.total() == 0) {
    // Cannot calculate anything if we don't have horizon homography.
    return;
  }

  cv::Mat horizonP = cv::Mat(cv::Point3d(frame.cols/2, frame.rows/2, 1));
  cv::Mat horizonP1;
  if (horizonH.total() == 0) {
    horizonP1 = horizonP;
  } else {
    horizonP1 = horizonH * horizonP;
    // Normalize the translated point.
    horizonP1 /= horizonP1.at<double>(2);
  }

  cv::calcOpticalFlowPyrLK(lastGray, gray, lastOdomPoints, odomPoints,
			   status, error, odomWindow);
  cv::Mat odomInliers;
  cv::Mat odomH = cv::findHomography(lastOdomPoints, odomPoints,
				     cv::RANSAC, 2.5F, odomInliers);
  cv::Mat odomP = cv::Mat(cv::Point3d(frame.cols/2,
				      frame.rows - odomMargin - odomHeight/2,
				      1));
  cv::Mat odomP1;
  if (odomH.total() == 0) {
    odomP1 = odomP;
  } else {
    odomP1 = odomH * odomP;
    odomP1 /= odomP1.at<double>(2);
  }

  cv::Point2f offset(1,1);
  cv::Scalar vectorColor(0, 255, 80);
  cv::Scalar consensusColor(0, 0, 255);

  for (int i=0; i < horizonPoints.size(); ++i) {
    if (horizonInliers.at<uchar>(i)) {
      cv::rectangle(frame, lastHorizonPoints[i]-offset,
		    lastHorizonPoints[i]+offset,
		    vectorColor);
      cv::line(frame, lastHorizonPoints[i], horizonPoints[i], vectorColor);
    }

    if (horizonH.total() > 0) {
      cv::Point2f p(horizonP.at<double>(0), horizonP.at<double>(1));
      cv::Point2f p1(horizonP1.at<double>(0), horizonP1.at<double>(1));
      cv::rectangle(frame, p1-offset, p1+offset, consensusColor);
      cv::line(frame, p, p1, consensusColor);
    }
  }

  int inlierCount = 0;
  for (int i=0; i < odomPoints.size(); ++i) {
    if (odomInliers.at<uchar>(i)) {
      ++inlierCount;
    }
  }

  if (inlierCount>=40 && inlierCount>=odomPoints.size()/5) {
    for (int i=0; i < odomPoints.size(); ++i) {
      if (odomInliers.at<uchar>(i)) {
	cv::rectangle(frame, lastOdomPoints[i]-offset,
		      lastOdomPoints[i]+offset,
		      vectorColor);
	cv::line(frame, lastOdomPoints[i], odomPoints[i], vectorColor);
      }
    }

    if (odomH.total() > 0) {
      cv::Point2f p(odomP.at<double>(0), odomP.at<double>(1));
      cv::Point2f p1(odomP1.at<double>(0), odomP1.at<double>(1));
      cv::rectangle(frame, p1-offset, p1+offset, consensusColor);
      cv::line(frame, p, p1, consensusColor);
    }
  }
  
  if (horizonH.total() > 0) {
    if (odomH.total() > 0) {
      publishOdometry(horizonP1.at<double>(0) - horizonP.at<double>(0),
		      horizonP1.at<double>(1) - horizonP.at<double>(1),
		      odomP1.at<double>(0) - odomP.at<double>(0),
		      odomP1.at<double>(1) - odomP1.at<double>(1));
    } else {
      publishOdometry(horizonP1.at<double>(0) - horizonP.at<double>(0),
		      horizonP1.at<double>(1) - horizonP.at<double>(1),
		      NAN, NAN);
    }      
  }

  if (publishFlow) {
    sensor_msgs::ImagePtr msg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    imagePub.publish(msg);
  }
}

void processImage(const sensor_msgs::Image::ConstPtr& msg) {
  cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;

  frameTime = msg->header.stamp;

  if (frameCount == 0) {
    makeMasks(frame);
  }

  ++frameCount;
  ROS_INFO("Processing frame %ld", frameCount);

  cv::Mat gray;
  cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

  std::vector<cv::KeyPoint> keypoints;
  std::vector<cv::Point2f> horizonPoints;
  std::vector<cv::Point2f> odomPoints;

  horizonDetector->detect(gray, keypoints, horizonMask);
  cv::KeyPoint::convert(keypoints, lastHorizonPoints);

  odomDetector->detect(gray, keypoints, odomMask);
  cv::KeyPoint::convert(keypoints, lastOdomPoints);

  if (frameCount >= 2) {
    findFlow(frame, gray, horizonPoints, odomPoints);
  }

  lastGray = gray;

  lastHorizonPoints = horizonPoints;
  lastOdomPoints = odomPoints;
}

/**
 * Starts the ROS node.
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "video_odometry");
  ros::NodeHandle n("~");

  n.param("frame_rate", frameRate, 15);

  n.param("horizon_width", horizonWidth, 400);
  n.param("horizon_height", horizonHeight, 150);

  n.param("odom_width", odomWidth, 300);
  n.param("odom_height", odomHeight, 150);
  n.param("odom_margin", odomMargin, 50);

  n.param("horizon_features", horizonFeatures, 500);
  n.param("horizon_quality", horizonQuality, 0.01);
  n.param("horizon_min_distance", horizonMinDistance, 1);
  n.param("horizon_window_size", horizonWindowSize, 21);

  n.param("odom_features", odomFeatures, 500);
  n.param("odom_quality", odomQuality, 0.01);
  n.param("odom_min_distance", odomMinDistance, 1);
  n.param("odom_window_size", odomWindowSize, 21);

  n.param("heading_scaling", headingScaling, 0.002);
  n.param("odom_scaling", odomScaling, 0.003);

  n.param("odom_frame", odomFrame, std::string("odom"));
  n.param("child_frame", childFrame, std::string("base_link"));

  n.param("publish_flow", publishFlow, false);
  
  ros::Subscriber imageSub = n.subscribe("/color/image_raw", 1000, processImage);

  odomPub = n.advertise<nav_msgs::Odometry>("/odom", 1000);
  imagePub = n.advertise<sensor_msgs::Image>("/color/image_flow", 1000);

  horizonDetector =
    cv::GFTTDetector::create(horizonFeatures, horizonQuality,
			     horizonMinDistance, 3);
  odomDetector =
    cv::GFTTDetector::create(odomFeatures, odomQuality,
			     odomMinDistance, 3);
  frameCount = 0;

  ros::spin();
  
  return 0;
}
