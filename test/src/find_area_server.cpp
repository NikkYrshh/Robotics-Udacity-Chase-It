#include "ros/ros.h"
#include "test/FindArea.h"

bool mltpl(test::FindArea::Request &req,
           test::FindArea::Response &res)
{
    res.area = req.height * req.width;
    ROS_INFO_STREAM(
    "request: " 
    << "height: " 
    << static_cast<double>(req.height) 
    << " width: " 
    << req.width << "\n");
    
    ROS_INFO_STREAM("back response >> area: " << static_cast<double>(res.area) << "\n");
    return true;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "find_area_server");
    
    ros::NodeHandle nh; 
    
    ros::ServiceServer service = nh.advertiseService("find_area", mltpl);

    ros::spin();
}

