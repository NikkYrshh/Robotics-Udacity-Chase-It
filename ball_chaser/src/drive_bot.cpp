#include <sstream>

#include "ball_chaser/DriveToTarget.h"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"

// Class to encapsulate the server
class DriveBotSrv {
 private:
  ros::Publisher motor_command_publisher;
  std::unique_ptr<ros::NodeHandle> nh;
  ros::ServiceServer service;

 public:
  DriveBotSrv() : nh(new ros::NodeHandle()) {
    motor_command_publisher =
        nh->advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    service = nh->advertiseService("ball_chaser/command_robot",
                                   &DriveBotSrv::handle_drive_request, this);
  }

  bool handle_drive_request(ball_chaser::DriveToTarget::Request &req,
                            ball_chaser::DriveToTarget::Response &res) {
    geometry_msgs::Twist motor_command;

    motor_command.linear.x = req.linear_x;
    motor_command.angular.z = req.angular_z;

    std::stringstream ss;
    ss << "Linear velocity: " << motor_command.linear.x
       << "; Angular velocity: " << motor_command.angular.z;

    res.msg_feedback = ss.str();
    ROS_INFO_STREAM(res.msg_feedback);

    motor_command_publisher.publish(motor_command);

    return true;
  }
};

int main(int argc, char **argv) {
  // Initialize a ROS node
  ros::init(argc, argv, "drive_bot");

  DriveBotSrv drive_bot;

  ros::spin();

  return 0;
}