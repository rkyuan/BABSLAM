#include "babs_slam.h"

// Table 5.3 from the book
// Samples a pose from the motion model given previous pose and control
// params assumed to be of length 6
geometry_msgs::Pose babs_slam::sampleMotionModel(geometry_msgs::Pose pose) {

	//ROS_INFO("start motion model");
	float poseAng = convertPlanarQuat2Phi(pose.orientation);
	
	float dx = last_odom.pose.pose.position.x-last_odom_used.pose.pose.position.x;
	float dy = last_odom.pose.pose.position.y-last_odom_used.pose.pose.position.y;
	float oldtheta = convertPlanarQuat2Phi(last_odom_used.pose.pose.orientation);
	float newtheta = convertPlanarQuat2Phi(last_odom.pose.pose.orientation);
	float trans = sqrt((dx*dx)+(dy*dy));

	float dtheta = min_dang(newtheta-oldtheta);
	float thetaprime = poseAng + dtheta + sample_normal(M0*fabs(dtheta));
	trans = trans + sample_normal(M1*trans);
	float xprime = pose.position.x + trans * cos(thetaprime);
	float yprime = pose.position.y + trans * sin(thetaprime);
	geometry_msgs::Pose result;



	// float rot1 = min_dang(atan2(dy,dx)-oldtheta);
	// float rot2 = min_dang(newtheta-oldtheta-rot1);
	// 

	// float rot1hat = rot1 + sample_normal(M0*fabs(rot1)+M1*trans);
	// float transhat = trans + sample_normal(M2*trans + M3*(fabs(rot1)+fabs(rot2)));
	// float rot2hat = rot2 + sample_normal(M4*fabs(rot2)+M5*trans);

	// float xprime = pose.position.x + transhat*cos(poseAng*rot1hat);
	// float yprime = pose.position.y + transhat*sin(poseAng*rot1hat);
	// float thetaprime = poseAng + rot1hat + rot2hat;

	
	result.position.x = xprime;
	result.position.y = yprime;
	result.orientation = convertPlanarPhi2Quaternion(thetaprime);

	return result;


	// geometry_msgs::Twist twist = last_odom.twist.twist;
	// ROS_INFO("odom before: x: %f, y: %f, ang: %f", last_odom.pose.pose.position.x+ROBOT_START_POSE_X, last_odom.pose.pose.position.y+ROBOT_START_POSE_Y, convertPlanarQuat2Phi(last_odom.pose.pose.orientation));
	// double fwd_vel = twist.linear.x;
	// double twist_vel = twist.angular.z;
	// double theta = convertPlanarQuat2Phi(pose.orientation);

	// // Add noise to the initial pose
	// double v = fwd_vel + sample_normal(M0*pow(fwd_vel,2) + M1*pow(twist_vel,2));
	// v = v/MAP_RESOLUTION; // convert from m/s to cells/s
	// double w = twist_vel + sample_normal(M2*pow(fwd_vel,2) + M3*pow(twist_vel,2));
	// double gamma = sample_normal(M4*pow(fwd_vel,2) + M5*pow(twist_vel,2));

	// // Calculate the final pose
	// if (fabs(w)>0.001) {
	// 	// w!=0
	// 	pose.position.x = pose.position.x - (v/w)*sin(theta) + (v/w)*sin(theta + w*dt);
	// 	pose.position.y = pose.position.y + (v/w)*cos(theta) - (v/w)*cos(theta + w*dt);
	// }
	// else {
	// 	// w==0, moving along a straight line
	// 	pose.position.x = pose.position.x - v*dt*cos(theta);
	// 	pose.position.y = pose.position.y + v*dt*sin(theta);
	// }
	// double newTheta = theta + w*dt + gamma*dt;

	// pose.orientation = convertPlanarPhi2Quaternion(newTheta);

	// ROS_INFO("odom after: x: %f, y: %f, ang: %f", pose.position.x, pose.position.y, convertPlanarQuat2Phi(pose.orientation));
	// return pose;
}

// Table 5.4 from the book
double babs_slam::sample_normal(double b) {
	double result = 0;
	for (int i = 1; i <= 12; i++) {
		//add a rand number between -b and b
		result += (static_cast<float>(rand())/(static_cast<float>(RAND_MAX/(2*b)))) - b;
	}
	return 0.5*result;
}