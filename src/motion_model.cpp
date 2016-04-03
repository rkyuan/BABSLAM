#include "babs_slam.h"

// Table 5.3 from the book
// Samples a pose from the motion model given previous pose and control
// params assumed to be of length 6
geometry_msgs::Pose babs_slam::sampleMotionModel(nav_msgs::Odometry state, double params[6]) {
	geometry_msgs::Pose pose = state.pose.pose;
	geometry_msgs::Twist twist = state.twist.twist;

	double fwd_vel = twist.linear.x;
	double twist_vel = twist.angular.z;
	double theta = convertPlanarQuat2Phi(pose.orientation);

	// Add noise to the initial pose
	double v = fwd_vel + sample_normal(params[0]*pow(fwd_vel,2) + params[1]*pow(twist_vel,2));
	v = v/MAP_RESOLUTION; // convert from m/s to cells/s
	double w = twist_vel + sample_normal(params[2]*pow(fwd_vel,2) + params[3]*pow(twist_vel,2));
	double gamma = sample_normal(params[4]*pow(fwd_vel,2) + params[5]*pow(twist_vel,2));

	// Calculate the final pose
	if (fabs(w)>0.001) {
		// w!=0
		pose.position.x = pose.position.x - (v/w)*sin(theta) + (v/w)*sin(theta + w*dt);
		pose.position.y = pose.position.y + (v/w)*cos(theta) + (v/w)*cos(theta + w*dt);
	}
	else {
		// w==0, moving along a straight line
		pose.position.x = pose.position.x - v*dt*cos(theta);
		pose.position.y = pose.position.y + v*dt*sin(theta);
	}
	double newTheta = theta + w*dt + gamma*dt;

	pose.orientation = convertPlanarPhi2Quaternion(newTheta);

	return pose;
}

// Table 5.4 from the book
double babs_slam::sample_normal(double bSquared) {
	double b = sqrt(bSquared);
	double result = 0;
	for (int i = 1; i <= 12; i++) {
		//add a rand number between -b and b
		result += (static_cast<float>(rand())/(static_cast<float>(RAND_MAX/(2*b)))) - b;
	}
	return 0.5*result;
}