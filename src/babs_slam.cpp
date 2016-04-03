#include "babs_slam.h"


babs_slam::babs_slam(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{ // constructor


	ROS_INFO("initializing subscribers");
	initializeSubscribers();
	initializePublishers();
	initializeParticles();
}

void babs_slam::initializeParticles(){
	particles.clear();
	nav_msgs::MapMetaData info;
	info.resolution = MAP_RESOLUTION;
	info.width = MAP_MAX_X;
	info.height = MAP_MAX_Y;

	geometry_msgs::Pose origin;
	origin.position.x = -10;
	origin.position.y = -10;

	info.origin = origin;

	for (int i = 0; i < NUMPARTICLES; i++){
		nav_msgs::OccupancyGrid map;
		map.header.frame_id = "map_frame";
		map.info = info;
		for (int j = 0; j < MAP_MAX_X*MAP_MAX_Y; j++){
			map.data[j]=50;
		}
	}
}


//this is the highest level of abstraction for the algorithm
void babs_slam::update(){
	ROS_INFO("updating");
	//THIS IS JUST TESTING---------------------------------------
	std::vector<float> particleWeights;
	//result will update particles
	for (int i = 0; i < particles.size();i++){
		geometry_msgs::Pose p = particles[i].pose;
		//get new particles
		geometry_msgs::Pose newpose = last_odom.pose.pose;
		particles[i].pose = newpose;
		//geometry_msgs::Pose newpose = sampleMotionModel(p);
		//weigh particles
		//particleWeights.push_back(measurementModelMap(newpose));
		updateMap(particles[i]);
	}
	
	//resample(particleWeights);
}


void babs_slam::resample(std::vector<float> weights){
	float total_weight = 0;
	for (int i = 0; i < NUMPARTICLES; i ++){
		//add up the size of all the weights
		total_weight+= weights[i];
	}
	std::vector<float> samples;
	for (int i = 0; i < NUMPARTICLES; i++){
		//take a bunch of random numbers in the range of the weights
		samples.push_back(rand()/RAND_MAX*total_weight);
	}
	std::sort(samples.begin(),samples.end());
	std::vector<particle> newParticles;
	total_weight =  weights[0];
	int weight_counter = 0;
	for (int i = 0; i < NUMPARTICLES; i++){
		if (total_weight>samples[i]){
			newParticles.push_back(particles[weight_counter]);
		}
		else{
			weight_counter ++;
			total_weight += weights[weight_counter];
		}
	}
	particles = newParticles;
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







