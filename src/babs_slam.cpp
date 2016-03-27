#include "babs_slam.h"


babs_slam::babs_slam(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{ // constructor
    
}

void babs_slam::update(){
	std::vector<geometry_msgs::Pose> newParticles;
	std::vector<float> particleWeights;
	//result will update particles
	for (int i = 0; i < particles.size();i++){
		geometry_msgs::Pose p = particles[i];
		geometry_msgs::Pose newpose = sampleMotionModel(p);


	}
}

geometry_msgs::Pose babs_slam::sampleMotionModel(geometry_msgs::Pose p){
	
	return p;
}

int main(int argc, char** argv) 
{
    

    ros::init(argc, argv, "babs_slam"); 

    ros::NodeHandle nh; 

    
    babs_slam babs(&nh); 

    
    ros::spin();
    return 0;
} 