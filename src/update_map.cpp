#include "babs_slam.h"

// table 9.1 from the book
void babs_slam::updateMap(particle p){
//	for all i,j {
//			if i,j is in the LIDAR measurement cone:
//				priorLogOdds = log(priorOcc/(1-priorOcc)); // eq 9.7
//				map[i][j] = map[i][j] + inverseSensorModel(i,j,measurement,pose) - priorLogOdds;
//		}
//		return map
}

//float inverseSensorModel
//// table 9.2 from the book
//// Computes the change in each map cell
////
//// x and y reference the cell of interest in the occupancy grid.
//// Measurement is the entire lidar scan
//// Pose is the Pose of the robot
//float inverseSensorModel(int x, int y, Measurement mt, Pose pose) {
//	// Get the center of the cell
//	float x = cellX + 0.5;
//	float y = cellY + 0.5;
//	// Get distance and angle of (cell-pose) vector
//	float r = sqrt((x-pose.x)^2+(y-pose.y)^2);
//	float phi = atan2(x-pose.x, y-pose.y) - pose.theta
//	// Find the closest ray
//	minAngle = 99999
//	k = 0
//	for (i=0; i<len(mt.values); i++) {
//		float angle = getAngle(pose, i);
//		if (angle < minAngle) {
//			minAngle = angle;
//			k = i;
//		}
//	}
//	z = mt.values[k]
//	// Change cell occupancy depending on the measurement and cell coordinates
//	if (r > min(zMax, z+ismAlpha/2) || abs(phi-minAngle)>ismBeta/2)
//		return l0;
//	if (z<zMax && abs(r-z)<ismAlpha/2)
//		return lOcc;
//	return lFree;
//}