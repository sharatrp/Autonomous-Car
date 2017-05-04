#pragma once
//A simple OpenCV3 node
#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/opencv_modules.hpp>
//#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <opencv2/core/ocl.hpp>

#include <image_transport/image_transport.h>

class SimpleNode
{
public:
  SimpleNode();
  ~SimpleNode();
  void process(const sensor_msgs::ImageConstPtr& msg);

private:
  ros::Publisher image_pub_;
 
  ros::NodeHandle n_;
  image_transport::ImageTransport it;
  image_transport::Subscriber image_sub_;

  int oclDev;
  int frameCount;
  
  cv::ocl::Context context;  
};


