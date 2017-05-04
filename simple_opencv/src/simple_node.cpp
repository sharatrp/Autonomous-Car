/* Simple demo node to show use of cv_bridge with OpenCL-enabled UMats */
#include <simple_node/simple_node.h>

using namespace std;
using namespace sensor_msgs;

SimpleNode::SimpleNode() : it(n_)
{
  ROS_INFO("Simple node started");
  image_pub_ = n_.advertise<sensor_msgs::Image>("annotated", 1);
  image_sub_ = it.subscribe("/camera/image_raw", 1, &SimpleNode::process, this);
  
  ros::NodeHandle local_ns_("~");
  local_ns_.param<int>("ocldev", oclDev, -1);
  ROS_INFO("OpenCL Dev?: %d Have openCL: %d", oclDev, cv::ocl::haveOpenCL());
  cout <<"Hello";
  if (oclDev > -1)
	{
	  //Try to verify that ocl device exists:
	  //Figure out if OpenCL is usable:
	  if (!cv::ocl::haveOpenCL())
	    {
	      cout << "OpenCL is not available..." << endl;
	      //return;
	    }

	  
	  if (!context.create(cv::ocl::Device::TYPE_GPU))
	    {
	      cout << "Failed creating the context..." << endl;
	      //return;
	    }

	  cout << context.ndevices() << " GPU devices are detected." << endl; //This bit provides an overview of the OpenCL devices you have in your computer
	  for (int i = 0; i < context.ndevices(); i++)
	    {
	      cv::ocl::Device device = context.device(i);
	      cout << "name:              " << device.name() << endl;
	      cout << "available:         " << device.available() << endl;
	      cout << "imageSupport:      " << device.imageSupport() << endl;
	      cout << "OpenCL_C_Version:  " << device.OpenCL_C_Version() << endl;
	      cout << endl;
	    }
	  if (oclDev > context.ndevices())
	    {
	      ROS_ERROR("Requested OpenCL device %d does not exist", oclDev);
	    }
	  cv::ocl::Device(context.device(oclDev)); //Here is where you change which GPU to use (e.g. 0 or 1)
	  cv::ocl::setUseOpenCL(true);
	}
      else
	{
	  cv::ocl::setUseOpenCL(false);
	}
      ROS_INFO("OpenCL Enabled?: %d", cv::ocl::useOpenCL());

      
}

SimpleNode::~SimpleNode() {}

void SimpleNode::process(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImageConstPtr cvImage;
  cv::UMat srcImage;

  //Get a CvShare from the ROS message
  cvImage = cv_bridge::toCvShare(msg);

  //(Potentially) copy it into GPU memory (a UMat)
  cvImage->image.copyTo(srcImage);

  //Create an RGB image to draw into
  cv::UMat rgbImage;
  if (msg->encoding == sensor_msgs::image_encodings::MONO8)
    cv::cvtColor(srcImage, rgbImage, cv::COLOR_GRAY2RGB);
  else
    rgbImage = srcImage;
  
  //Draw into the output image:
  cv::Point2i pt(msg->width/2, msg->height/2);
  cv::circle(rgbImage, pt, 10, cv::Scalar(0, 255, 0),3);
  
  // Send the output image over ROS
  sensor_msgs::Image img_msg;
  cv_bridge::CvImage img_bridge;

  std_msgs::Header hdr;
  hdr.seq = frameCount++; 
  hdr.stamp = ros::Time::now();
  img_bridge = cv_bridge::CvImage(hdr, sensor_msgs::image_encodings::BGR8, rgbImage.getMat(cv::ACCESS_READ));
  img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image

  image_pub_.publish(img_msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_node");
  SimpleNode theNode;
  ros::spin();
  return 0;
}
