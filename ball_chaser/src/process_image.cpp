#include <sensor_msgs/Image.h>

#include "ball_chaser/DriveToTarget.h"
#include "ros/ros.h"

class DriveBotClient {
  // Define a global client that can request services
 private:
  ros::ServiceClient client;
  std::unique_ptr<ros::NodeHandle> nh;
  ros::Subscriber sub1;

 public:
  DriveBotClient() : nh(new ros::NodeHandle) {
    client = nh->serviceClient<ball_chaser::DriveToTarget>(
        "/ball_chaser/command_robot");
    sub1 = nh->subscribe<sensor_msgs::Image>(
        "/camera/rgb/image_raw", 10,
        [this](const sensor_msgs::ImageConstPtr& img) {
          this->process_image_callback(*img);
        });
  }

  // This function calls the command_robot service to drive the robot in the
  // specified direction
  void drive_robot(float lin_x, float ang_z) {
    // TODO: Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    if (client.call(srv)) {
      ROS_INFO_STREAM("Requested: " << srv.response.msg_feedback);

    } else {
      ROS_ERROR("Failed to call the service");
    }
  }

  // This callback function continuously executes and reads the image data
  void process_image_callback(const sensor_msgs::Image& img) {
    int white_pixel = 255;
    bool wp_flag = false;

    for (int i = 0; i < (img.height * img.step - 3); i += 3) {
      if (img.data[i] == white_pixel && img.data[i + 1] == white_pixel &&
          img.data[i + 2] == white_pixel) {
        // White pixel found
        wp_flag = true;
        int col = (i + 1) % img.width;
        int seg_len = img.width / 3;

        if (col < seg_len) {
          // Left segment
          ROS_INFO_STREAM("The white pixel on the left");

          drive_robot(0.1, 0.5);
          return;

        } else if (col < seg_len * 2) {
          // Center segment
          ROS_INFO_STREAM("The white pixel in front");

          drive_robot(0.5, 0);
          return;

        } else {
          // Right segment
          ROS_INFO_STREAM("The white pixel on the right");
          drive_robot(1.0, -0.5);
          return;
        }
      }
    }

    if (!wp_flag) {
      // Stop
      ROS_INFO_STREAM("Stopping!");

      drive_robot(0, 0);
    }
    ros::Duration(0.1).sleep();
  }
};
int main(int argc, char** argv) {
  // Initialize the process_image node and create a handle to it
  ros::init(argc, argv, "process_image");

  // Define a client service capable of requesting services from command_robot

  // Subscribe to /camera/rgb/image_raw topic to read the image data inside the
  // process_image_callback function
  DriveBotClient client_instance;
  // Handle ROS communication events
  ros::spin();

  return 0;
}
