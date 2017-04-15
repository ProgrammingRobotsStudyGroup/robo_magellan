// video_publish.cpp - ROS node to publish video frames to a ROS topic.

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <iostream>

/**
 * Starts the ROS node.
 */
int main(int argc, char **argv) {
  int frameRate;
  std::string videoFile;
  
  ros::init(argc, argv, "video_publish");
  ros::NodeHandle n("~");

  n.param("frame_rate", frameRate, 15);
  if (!n.getParam("file", videoFile)) {
    std::cerr << "Video file not set" << std::endl;
    return 1;
  }

  ros::Publisher imagePub = n.advertise<sensor_msgs::Image>("/color/image_raw", 1000);

  ros::Rate loop_rate(frameRate);

  ROS_INFO("Video file %s", videoFile.c_str());
  ROS_INFO("Frame rate %d", frameRate);

  cv::VideoCapture video(videoFile);
  if(!video.isOpened()) {
    ROS_INFO("Could not open video file: %s", videoFile.c_str());
    return 1;
  }

  cv::Mat frame;

  ROS_INFO("Got here");

  long frameCount = 0;
  while (ros::ok()) {
    if (!video.read(frame)) {
      ROS_INFO("End of video file");
      break;
    }
    ++frameCount;

    sensor_msgs::ImagePtr msg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    imagePub.publish(msg);
    ROS_INFO("Published frame %ld", frameCount);
    
    loop_rate.sleep();
  }

  return 0;
}
