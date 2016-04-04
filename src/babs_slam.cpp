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
	origin.position.x = -1.0*ROBOT_START_POSE_X;//-30;
	origin.position.y = -1.0*ROBOT_START_POSE_Y;//-30;

	info.origin = origin;
	ROS_INFO("here");
	geometry_msgs::Pose p;
	p.position.x = 0;
	p.position.y = 0;
	p.orientation = convertPlanarPhi2Quaternion(0);
	for (int i = 0; i < NUMPARTICLES; i++){

		nav_msgs::OccupancyGrid map;
		map.header.frame_id = "map_frame";
		map.info = info;
		map.data.resize(MAP_MAX_X*MAP_MAX_Y);
		for (int j = 0; j < MAP_MAX_X*MAP_MAX_Y; j++){
			map.data[j]=50;
		}
		particle part;
		part.pose = p;
		part.map = map;
		particles.push_back(part);
	}
}


//this is the highest level of abstraction for the algorithm
void babs_slam::update(){
	ROS_INFO("updating");
	std::vector<float> particleWeights;
	
	//result will update particles
	for (int i = 0; i < particles.size();i++){
		
		geometry_msgs::Pose oldpose = particles[i].pose;
		//get new particles

		geometry_msgs::Pose newpose;
		newpose = sampleMotionModel(oldpose);
		particles[i].pose = newpose;
		
		
		

		if (i == 0) {

			ROS_INFO("broadcasting transform");


			geometry_msgs::Quaternion odom_quat = newpose.orientation;
			double phi = convertPlanarQuat2Phi(odom_quat);

			//publish the transform over tf
			geometry_msgs::TransformStamped odom_trans;
			odom_trans.header.stamp = ros::Time::now();
			//			odom_trans.header.frame_id = "base_laser1_link";
			//			odom_trans.child_frame_id = "base_link";

			odom_trans.header.frame_id = "base_laser1_link";
			odom_trans.child_frame_id = "map_frame";

			//			odom_trans.header.frame_id = "map_frame";
			//			odom_trans.child_frame_id = "base_laser1_link";

			double x = newpose.position.x;
			double y = newpose.position.y;

			ROS_INFO("x,y is %f %f", x, y);

			double angle = 2*M_PI - phi;

			odom_trans.transform.translation.x = -x*cos(angle) + y*sin(angle);
			odom_trans.transform.translation.y = -x*sin(angle) - y*cos(angle);
			odom_trans.transform.translation.z = 0.0;

			odom_trans.transform.rotation = convertPlanarPhi2Quaternion(-phi);

			//send the transform
			odom_broadcaster.sendTransform(odom_trans);

		}


		updateMap(particles[i]);

		//weigh particles
		float weight = 1;
		weight *= measurementModelMap(newpose,particles[i].map);
		
		float w2 = imuModel(newpose,oldpose);
		weight *= w2;
		
		particleWeights.push_back(weight);

		//ROS_INFO("particle pose %f %f", particles[i].pose.position.x, particles[i].pose.position.y);
		
	}

	updateLastMeasurements();

	
	resample(particleWeights);
	
	map_publisher.publish(particles[(rand()%(int)(NUMPARTICLES))].map);
	
}


void babs_slam::resample(std::vector<float> weights){
	float total_weight = 0;
	for (int i = 0; i < NUMPARTICLES; i ++){
		//add up the size of all the weights
		total_weight+= weights[i];
		//ROS_INFO("x: %f y: %f weight: %f", particles[i].pose.position.x, particles[i].pose.position.y, weights[i]);
	}
	std::vector<float> samples;
	for (int i = 0; i < NUMPARTICLES; i++){
		//take a bunch of random numbers in the range of the weights
		samples.push_back(static_cast<float>(rand())/(static_cast<float>(RAND_MAX/total_weight)));
		//samples.push_back(rand()/RAND_MAX*total_weight);
	}
	std::sort(samples.begin(),samples.end());
	std::vector<particle> newParticles;
	total_weight =  weights[0];
	int weight_counter = 0;
	for (int i = 0; i < NUMPARTICLES; i++){
		if (total_weight>samples[i] || weight_counter==NUMPARTICLES){
			newParticles.push_back(particles[weight_counter]);
			ROS_INFO("resample %d", weight_counter);
		}
		else{
			weight_counter ++;
			total_weight += weights[weight_counter];
			i--;
		}
	}
	particles = newParticles;
}








