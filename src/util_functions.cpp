#include "babs_slam.h"


// a useful conversion function: from quaternion to yaw
double babs_slam::convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion) {
	double quat_z = quaternion.z;
	double quat_w = quaternion.w;
	double phi = 2.0 * atan2(quat_z, quat_w); // cheap conversion from quaternion to heading for planar motion
	return phi;
}

//and the other direction:
geometry_msgs::Quaternion babs_slam::convertPlanarPhi2Quaternion(double phi) {
	geometry_msgs::Quaternion quaternion;
	quaternion.x = 0.0;
	quaternion.y = 0.0;
	quaternion.z = sin(phi / 2.0);
	quaternion.w = cos(phi / 2.0);
	return quaternion;
}

double babs_slam::min_dang(double dang) {
    while (dang > M_PI) dang -= 2.0 * M_PI;
    while (dang < -M_PI) dang += 2.0 * M_PI;
    return dang;
}


bool babs_slam::compareFloats(float a, float b) {
	if (fabs(a-b) < 0.001)
		return true;
	return false;
}

// Map helper functions
// Gets int8 value of the cell (x,y) given row-major representation
int babs_slam::map_get_value(nav_msgs::OccupancyGrid map, int x, int y) {
	int index = x + y*MAP_MAX_X;
	return map.data[index];
}

// Check if (x,y) is within map bounds
bool babs_slam::within_map_bounds(int x, int y) {
	return (0<=x && x<MAP_MAX_X && 0<=y && y<MAP_MAX_Y);
}

// Takes probability 0-100 and returns log odds representation
float babs_slam::prob_to_log_odds(int prob) {
	return log(prob/(100-prob));
}

// Takes log odds representation and returns the probability 0-100
int babs_slam::log_odds_to_prob(float logOdds) {
	return 100*exp(logOdds)/(1+exp(logOdds));
}