#ifndef PROCESS_H_
#define PROCESS_H_

// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------
#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_calibration_parsers/parse.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <sstream>
#include <termios.h>
#define SIZE 640*480
#define CALI_NUM 5

#include "imu.h"
#include "slamBase.h"
#include "serial_port.h"
// ------------------------------------------------------------------------------
//   Defines
// ------------------------------------------------------------------------------
typedef unsigned long 	UInt64;
typedef long 			Int64;

// ------------------------------------------------------------------------------
//   Prototypes
// ------------------------------------------------------------------------------
static const std::string OPENCV_WINDOW = "Image window";
static const std::string OPENCV_DEPTH_WINDOW = "depth window";



void* thread_start(void *arg);
class ImageConverter
{
public:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Subscriber image_depth_sub_;
  image_transport::Publisher image_pub_,depth_image_pub_;
  ros::Subscriber drone_pose;

  IMU * _imu;
  Serial_Port * _serial_port;
  unsigned char ch;

  ImageConverter();
  ~ImageConverter();
  void poseCb(const geometry_msgs::PoseStampedPtr& msg);
  void imageCb(const sensor_msgs::ImageConstPtr& msg);
  void depth_imageCb(const sensor_msgs::ImageConstPtr& msg);
  void getkey();
  void process();
  void loop();
  
  int save_count;
  pthread_t read_tid;
  FRAME frame[CALI_NUM];
  Eigen::Matrix3d solve_axxb(Eigen::Matrix3d A, Eigen::Matrix3d B);
  Eigen::Matrix3d solve_two_frame(FRAME frame1, FRAME frame2, Eigen::Matrix3d mat1, Eigen::Matrix3d mat2);
  Eigen::Vector3d cnb2att(Eigen::Matrix3d trans_mat);

private:
	ParameterReader pd;
	cv_bridge::CvImagePtr cv_ptr,cv_depth_ptr;
	Eigen::Quaternion<double> cali_orient[CALI_NUM];
	Eigen::Matrix3d cali_mat[CALI_NUM];
	geometry_msgs::Quaternion orientation;
	CAMERA_INTRINSIC_PARAMETERS camera;
};

#endif // ROS_COMM_H_
