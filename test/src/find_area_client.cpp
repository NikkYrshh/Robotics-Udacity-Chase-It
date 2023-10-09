#include "ros/ros.h"
#include "test/FindArea.h"
#include <string>

int main(int argc, char **argv){

ros::init(argc, argv, "find_area_client");

ros::NodeHandle nh;
ros::ServiceClient client = nh.serviceClient<test::FindArea>("find_area");
test::FindArea srv;
srv.request.width = std::stod(argv[1]);
srv.request.height = std::stod(argv[2]);

if(client.call(srv)){
    ROS_INFO_STREAM("Area is: " << static_cast<double>(srv.response.area) << "\n");
}else{
    ROS_ERROR_STREAM("Failed to call service! :( " << "\n");
    return 1;
}

}

