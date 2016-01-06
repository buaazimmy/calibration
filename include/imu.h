#ifndef IMU_H_
#define IMU_H_

#include <iostream>
#include <stdbool.h>
#include <stdio.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <time.h>
#include <sys/time.h>

#define	Rad2Deg 	57.2957795130823208767981548141052
#define	Deg2Rad 	0.0174532922222222

typedef unsigned char uint8_t;
class IMU {
public:
	IMU();
	~IMU();
	struct imu_data {
		int16_t accl[3];
		int16_t gyro[3];
		int16_t mag[3];
		int16_t yaw;
		int16_t pitch;
		int16_t roll;
		int32_t presure;
	};
	Eigen::Matrix3d cib;
	Eigen::Quaternion<double> q,q_last;
	Eigen::Quaternion<double> rk;
	Eigen::Vector3d gyro;
	Eigen::Vector3d angle;
	Eigen::Vector3d acc;
	Eigen::Vector3d mag;
	Eigen::Vector3d att;

	imu_data data;
	time_t time;
	float interval;
	struct timespec time_start,time_end;
//	typedef imu_data* imu_data_t;
	int rev_process(char ch);
private:
	unsigned char rev_buf[64]; /* 状态机常量 */
	unsigned char crc;
	enum input_status {
		STATUS_IDLE, /* 空闲 */
		STATUS_SOF, /* 接收到帧头 (0x88, 0xAF)*/
		STATUS_LEN, /* 接收到长度字节 */
		STATUS_DATA, /*接收到数据 */
	};
	bool first_frame;
	int imu_rev_get_data();
	void compute_attitude();
};
#endif
