#include "babs_slam.h"


babs_slam::babs_slam(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{ // constructor
    
}


// Table 5.3 from the book
// Samples a pose from the motion model given previous pose and control
// params assumed to be of length 6
geometry_msgs::Pose babs_slam::sampleMotionModel(nav_msgs::Odometry state, double params[]) {
	// Add noise to the initial pose

	geometry_msgs::Pose pose = state.pose.pose;
	geometry_msgs::Twist twist = state.twist.twist;

	double fwd_vel = twist.linear.x;
	double twist_vel = twist.angular.z;
	double theta = convertPlanarQuat2Phi(pose.orientation);

	double v = fwd_vel + sample_normal(params[0]*pow(fwd_vel,2) + params[1]*pow(twist_vel,2));
	double w = twist_vel + sample_normal(params[2]*pow(fwd_vel,2) + params[3]*pow(twist_vel,2));

	// Calculate the final pose
	double gamma = sample_normal(params[4]*pow(fwd_vel,2) + params[5]*pow(twist_vel,2));
	pose.position.x = pose.position.x - (v/w)*sin(theta) + (v/w)*sin(theta + w*dt);
	pose.position.y = pose.position.y + (v/w)*cos(theta) + (v/w)*cos(theta + w*dt);
	double newTheta = theta + w*dt + gamma*dt;

	pose.orientation = convertPlanarPhi2Quaternion(newTheta);

	return pose;
}

// Table 5.4 from the book
double babs_slam::sample_normal(double bSquared) {
	double b = sqrt(bSquared);
	int result = 0;
	for (int i = 1; i <= 12; i++) {
		result += rand()*2*b - b;//add a rand number between -b and b
	}
	return 0.5*result;
}

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


int main(int argc, char** argv) 
{
    

    ros::init(argc, argv, "babs_slam"); 

    ros::NodeHandle nh; 

    
    babs_slam babs(&nh); 

    
    ros::spin();
    return 0;
} 
