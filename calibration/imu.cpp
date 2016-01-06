#include "imu.h"

IMU::IMU(){
	cib << 1.0,0.0,0.0,
			0.0,1.0,0.0,
			0.0,0.0,1.0;
	q = {1.0,0.0,0.0,0.0};
	q_last = {1.0,0.0,0.0,0.0};
	crc = 0;
	interval = 0;
	first_frame = true;
	time_start={0, 0};time_end={0, 0};
	clock_gettime(CLOCK_REALTIME, &time_start);
}
IMU::~IMU(){}
int IMU::rev_process(char ch) {
	int rec_flag = 0;
	static int len = 0;
	static int i; /* 状态机 */
	static enum input_status status = STATUS_IDLE; /* running status machine */
	switch (status) {
	case STATUS_IDLE:
		if ((uint8_t) ch == 0x88) {
			status = STATUS_SOF;
		}
		break;
	case STATUS_SOF:
		if ((uint8_t) ch == 0xAF) {
			status = STATUS_LEN;
		}
		break;
	case STATUS_LEN:
		len = ch;
		status = STATUS_DATA;
		i = 0;
		break;
	case STATUS_DATA:
		if (i == (len+1)) {
			status = STATUS_IDLE; /* 完整的接收完一帧数据,调用解析函数 */

			crc = (0x88+0xAF+28) & 0xFF;
			for (int j = 0; j < 27; j++)
				crc += rev_buf[j];
			if (crc == rev_buf[28]) {
				imu_rev_get_data();
				rec_flag = 1;
			}
			else
				printf("IMU crc err:%d,%d",crc,rev_buf[28]);
			break;
		}
		rev_buf[i++] = ch;
		break;
	default:
		break;
	}
	return rec_flag;
}
int IMU::imu_rev_get_data() {
	data.accl[0] = (rev_buf[0] << 8) + rev_buf[1];//0.001g
	data.accl[1] = (rev_buf[2] << 8) + rev_buf[3];
	data.accl[2] = (rev_buf[4] << 8) + rev_buf[5];
	data.gyro[0] = (rev_buf[6] << 8) + rev_buf[7];//0.001*0.06deg/s
	data.gyro[1] = (rev_buf[8] << 8) + rev_buf[9];
	data.gyro[2] = (rev_buf[10] << 8) + rev_buf[11];
	data.mag[0] = (rev_buf[12] << 8) + rev_buf[13];//uT
	data.mag[1] = (rev_buf[14] << 8) + rev_buf[15];
	data.mag[2] = (rev_buf[16] << 8) + rev_buf[17];
	data.roll = (rev_buf[18] << 8) + rev_buf[19];
	data.pitch = (rev_buf[20] << 8) + rev_buf[21];
	data.yaw = (rev_buf[22] << 8) + rev_buf[23];
	data.presure = (rev_buf[27] << 24) + (rev_buf[26] << 16)
			+ (rev_buf[25] << 8) + (rev_buf[24] << 0);

	for(int i=0;i<3;i++){
		gyro[i] = (double)data.gyro[i]*0.06;
		acc[i] = (double)data.accl[i]*0.001;
		mag[i] = data.mag[i];
	}
	att[0] 	= (double)data.yaw/10;
	att[1]= (double)data.pitch/100;
	att[2]= (double)data.roll/100;
//		printf("ACC: %2.4f\t%2.4f\t%2.4f\t", acc[0],acc[1],acc[2]);
//		printf("W:%2.4f\t%2.4f\t%2.4f\t",gyro[0],gyro[1],gyro[2]);
//		printf("Mag:%2.4f\t%2.4f\t%2.4f\t",mag[0],mag[1],mag[2]);
//		printf("YPR: %2.4f\t%2.4f\t%2.4f\n", yaw,pitch,roll);
	if(first_frame){
		first_frame = false;
		clock_gettime(CLOCK_REALTIME, &time_end);
		interval = time_end.tv_sec-time_start.tv_sec + (time_end.tv_nsec-time_start.tv_nsec)/1000000000;
		time_start = time_end;
	}
	else{
		compute_attitude();
	}

	return 0;
}
void IMU::compute_attitude(){
//		std::cout<<interval<<std::endl;
	clock_gettime(CLOCK_REALTIME, &time_end);
	interval = time_end.tv_sec-time_start.tv_sec + (time_end.tv_nsec-time_start.tv_nsec)/1000000000;
	time_start = time_end;
	angle = gyro * interval * Deg2Rad;
    double u = sqrt(angle.x()*angle.x()+angle.y()*angle.y()+angle.z()*angle.z());
    if(abs(u)<0.001)u=0.0001;
    double ac = cos(u/2);
    double as = sin(u/2)/u;

    rk = {ac,as*angle.x(),as*angle.y(),as*angle.z()};
    q = q * rk;
    q = q.normalized();
//    printf("q:%lf\t%lf\t%lf\t%lf\n",q.x(),q.y(),q.z(),q.w());
    cib = q.matrix();
}
