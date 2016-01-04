#include "imu.h"

void init_imu()
{
	_imu = new IMU;
	_serial_port = new Serial_Port;
	_serial_port->start();
}

void del_imu()
{
	_serial_port->stop();
}


//============================================================
// MAIN LOOP
//============================================================
void ImuThread::mainLoop()
{
	//TODO core dumped happened sometimes
	if(_serial_port->read_port(ch)<=0)
		UDEBUG("\nSerial port get data error\n");
	if (_imu->rev_process(ch)) {
		this->post(new ImuEvent(*_imu, " "));
	}
}
