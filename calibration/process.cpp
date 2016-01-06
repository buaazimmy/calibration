/**
 * @file process.cpp
 *
 *
 */

#include "process.h"

struct termios cooked, raw;
char c;
int kfd ;
bool dirty;

uint64_t get_time_usec()
{
	static struct timeval _time_stamp;
	gettimeofday(&_time_stamp, NULL);
	return _time_stamp.tv_sec*1000000 + _time_stamp.tv_usec;
}
// ------------------------------------------------------------------------------
//   Con/De structors
// ------------------------------------------------------------------------------
ImageConverter::ImageConverter()
  : it_(nh_)
{
	_imu = new IMU;
	_serial_port = new Serial_Port;
	_serial_port->start();
  // Subscrive to input video feed and publish output video feed
  image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1,
    &ImageConverter::imageCb, this);
  image_depth_sub_ = it_.subscribe("/camera/depth/image_raw", 1,
		    &ImageConverter::depth_imageCb, this);
  image_pub_ = it_.advertise("/image_converter/output_video", 1);
  depth_image_pub_ = it_.advertise("/image_converter/output_depth_video", 1);
//  imu_pose_pub_ = nh_.advertise("/image_converter/imu_pose",1);
  //const geometry_msgs::PoseStampedPtr& msg
  drone_pose = nh_.subscribe("/mavros/local_position/pose",1,&ImageConverter::poseCb,this);

  cv::namedWindow(OPENCV_WINDOW);
  cv::namedWindow(OPENCV_DEPTH_WINDOW);
  save_count=0;
  int kfd = 0;
  bool dirty = false;
  //get the console in raw mode
  	tcgetattr(kfd, &cooked);
  	memcpy(&raw, &cooked, sizeof(struct termios));
  	raw.c_lflag &=~ (ICANON | ECHO);
  	// Setting a new line, then end of file
  	raw.c_cc[VEOL] = 1;
  	raw.c_cc[VEOF] = 2;
  	tcsetattr(kfd, TCSANOW, &raw);

  	pthread_create(&read_tid, NULL, &thread_start,this);
  	pthread_create(&read_imu, NULL, &imu_loop,this);
}
ImageConverter::~ImageConverter()
{
	_serial_port->stop();
  cv::destroyWindow(OPENCV_WINDOW);
  cv::destroyWindow(OPENCV_DEPTH_WINDOW);
}
void ImageConverter::poseCb(const geometry_msgs::PoseStampedPtr& msg)
{
	geometry_msgs::Pose pose = msg->pose;
	orientation = pose.orientation;
//	ROS_INFO("%f\t%f\t%f\t%f\n",orientation.x,orientation.y,orientation.z,orientation.w);

}
void ImageConverter::depth_imageCb(const sensor_msgs::ImageConstPtr& msg)
{

  try
  {
	  cv_depth_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Update GUI Window
  cv::imshow(OPENCV_DEPTH_WINDOW, cv_depth_ptr->image);
  cv::waitKey(3);

  // Output modified video stream
  depth_image_pub_.publish(cv_depth_ptr->toImageMsg());
}
void ImageConverter::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Update GUI Window
  cv::imshow(OPENCV_WINDOW, cv_ptr->image);
  cv::waitKey(3);

  // Output modified video stream
  image_pub_.publish(cv_ptr->toImageMsg());

  pose.orientation.w = _imu->q.w();
  pose.orientation.x = _imu->q.x();
  pose.orientation.y = _imu->q.y();
  pose.orientation.z = _imu->q.z();

//  imu_pose_pub_.publish(&pose);

}
void ImageConverter::getkey()
{
	while(save_count<CALI_NUM){
		if(read(kfd, &c, 1) < 0)
		{
		  perror("read():");
		  exit(-1);
		}

		if(c=='s'){
			frame[save_count].rgb = cv_ptr->image;
			frame[save_count].depth = cv_depth_ptr->image;
			frame[save_count].frameID = save_count;
			Eigen::Quaternion<double> tmp=_imu->q;
			cali_mat[save_count] = tmp.matrix();
			save_count++;

			cout<<"Get pic"<<save_count<<endl;
			cout<<"Orientation:"<<endl<<tmp.x()<<"\t"<<tmp.y()<<"\t"<<tmp.z()<<"\t"<<tmp.w()<<"\t"<<endl;

			char name_path[30] = {0};
			sprintf(name_path,"~/catkin_ws/src/calibration/data/rgb%d.png",save_count);
			cv::imwrite( name_path, cv_ptr->image );
			sprintf(name_path,"~/catkin_ws/src/calibration/data/depth%d.png",save_count);
			cv::imwrite( name_path, cv_depth_ptr->image );

		}
	}
}
void ImageConverter::process(){
	Eigen::Matrix3d *mattest2 = new Eigen::Matrix<double,3,3>;
Eigen::Matrix3d *mattest = new Eigen::Matrix<double,3,3>;
///{1,2,3,4,5,6,7,8,9}
delete mattest;
	//	cout<<"extracting features"<<endl;
    string detecter = pd.getData( "detector" );
    string descriptor = pd.getData( "descriptor" );
    for(int i=0;i<CALI_NUM;i++)
    	computeKeyPointsAndDesp( frame[i], detecter, descriptor);

	// 相机内参
    camera.fx = atof( pd.getData( "camera.fx" ).c_str());
    camera.fy = atof( pd.getData( "camera.fy" ).c_str());
    camera.cx = atof( pd.getData( "camera.cx" ).c_str());
    camera.cy = atof( pd.getData( "camera.cy" ).c_str());
    camera.scale = atof( pd.getData( "camera.scale" ).c_str() );

    int i=0;
    int j=1;
    Eigen::Matrix3d relative_att = solve_two_frame(frame[i],frame[j],cali_mat[i],cali_mat[j]);
	cout<<"Relative attitude between IMU and camera:"<<endl<<relative_att<<endl;


}
Eigen::Matrix3d ImageConverter::solve_axxb(Eigen::Matrix3d A, Eigen::Matrix3d B){
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolverA(A);
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolverB(B);
	Eigen::Matrix3d X = Eigen::MatrixXd::Zero(3,3);
	 if((eigensolverA.info() == Eigen::Success)&(eigensolverB.info() == Eigen::Success)) {
		 X = eigensolverA.eigenvectors() * eigensolverB.eigenvectors().inverse();
	 }
	 else
		 cout<<"err: cannot solve eigenvector." <<endl;
	 return X;
}
Eigen::Matrix3d ImageConverter::solve_two_frame(FRAME frame1, FRAME frame2, Eigen::Matrix3d mat1, Eigen::Matrix3d mat2){
	//	cout<<"solving pnp"<<endl;
		// 求解pnp
		RESULT_OF_PNP result = estimateMotion( frame1, frame2, camera );

	//	cout<<"rvec: "<<endl<<result.rvec<<endl<<"tvec: "<<endl<<result.tvec<<endl;
		// 处理result
		// 将旋转向量转化为旋转矩阵
		cv::Mat R;
		cv::Rodrigues( result.rvec, R );
		Eigen::Matrix3d r;
		cv::cv2eigen(R, r);

		// 将平移向量和旋转矩阵转换成变换矩阵
		Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

		Eigen::AngleAxisd angle(r);
	//	cout<<"translation"<<endl<<r<<endl;

		Eigen::Matrix3d rotate_mat = mat1 * mat2.inverse();
	//	cout<<"Rotate_mat:"<<endl<<rotate_mat<<endl;
		rotate_mat.eigenvalues();

		return solve_axxb(r,rotate_mat);
}
Eigen::Vector3d ImageConverter::cnb2att(Eigen::Matrix3d trans_mat){
	Eigen::Vector3d att;
	double tmp1,tmp2;
	att[0] = asin(trans_mat(1,2));

	tmp1 = trans_mat(0,2);
	tmp2 = trans_mat(2,2);
	if ((tmp1<=0) && (tmp2>0))
		att[1] = atan(-tmp1/tmp2);
	else if ((tmp1<=0) && (tmp2 < 0))
		att[1] = atan(-tmp1/tmp2) + 3.1415926;
	else if ((tmp1>=0) && (tmp2>0))
		att[1] = atan(-tmp1/tmp2);
	else if ((tmp1>=0) && (tmp2<0))
		att[1] = atan(-tmp1/tmp2) - 3.1415926;
	else if ((tmp2 == 0) && (tmp1>0))
		att[1] = 3.1415926 * 1.5;
	else
		att[1] = 3.1415926/2;

	tmp1 = trans_mat(1,1);
	tmp2 = trans_mat(1,0);
	if ((tmp1>0) && (tmp2<=0))
		att[2] = atan((-tmp2)/tmp1);
	else if ((tmp1>0) && (tmp2>=0))
		att[2] = atan((-tmp2)/tmp1)+2*3.1415926;
	else if ((tmp1<0)&&(tmp2<=0))
		att[2] = atan((-tmp2)/tmp1)+3.1415926;
	else if ((tmp1<0) && (tmp2>=0))
		att[2] = atan((-tmp2)/tmp1)+3.1415926;
	else if ((tmp1 == 0) && (tmp2>0))
		att[2] = 3*3.1415926/2;
	else
		att[2] = 3.1415926/2;

	return att;

}
void* thread_start(void *arg)
{
	ImageConverter *ic_ = (ImageConverter *)arg;
	ic_->getkey();
	ic_->process();

	ros::shutdown();
}
void* imu_loop(void *arg){

	ImageConverter *ic_ = (ImageConverter *)arg;
	unsigned char ch;

	while(ros::ok()){
		if(ic_->_serial_port->read_port(ch)<=0)
			printf("Serial port get data error\n");
		if (ic_->_imu->rev_process(ch)) {
//			printf("roll:%ld\n",_imu->data.roll);
		}
	}
	ros::shutdown();
}
