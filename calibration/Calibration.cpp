#include "Calibration.h"
#include <iostream>
#include "process.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Calibration");

	ImageConverter ic;

	ros::spin();

	return 0;
}
