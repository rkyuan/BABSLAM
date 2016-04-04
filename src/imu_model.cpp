#include "babs_slam.h"

float babs_slam::imuModel(geometry_msgs::Pose newpose, geometry_msgs::Pose oldpose){
	float sigma = 0.15;
	float pi = 3.141592653;
	float newAng = convertPlanarQuat2Phi(newpose.orientation);
	float oldAng = convertPlanarQuat2Phi(oldpose.orientation);

	float dang = newAng - oldAng;

	dang = min_dang(dang);

	float newImu = convertPlanarQuat2Phi(last_imu.orientation);
	float oldImu = convertPlanarQuat2Phi(last_imu_used.orientation);

	float dimu = newImu - oldImu;

	dimu = min_dang(dimu);

	float error = dang - dimu;
	float weight = 1.0/(sigma*sqrt(2*pi));
	weight *= exp(-1.0*(error*error)/(2*sigma*sigma));


	return weight;


}