#include "babs_slam.h"


babs_slam::babs_slam(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{ // constructor
    
}

int main(int argc, char** argv) 
{
    

    ros::init(argc, argv, "babs_slam"); 

    ros::NodeHandle nh; 

    
    babs_slam babs(&nh); 

    
    ros::spin();
    return 0;
} 