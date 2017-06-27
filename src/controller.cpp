#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <obstacle_detector/Obstacles.h>

double x_d, y_d,theta_d;

double forward_control = 0.0;
double orientation_control = 0.0;
double k_pf, k_po;




void poseCallback(const geometry_msgs::Pose::ConstPtr& pose_msg)
{
  double x_error = x_d - pose_msg->x;
  double y_error = y_d - pose_msg->y;
  double theta_d = atan2(y_error, x_error);
  double theta_error = theta_d - pose_msg->theta;
  forward_control = 1.0;
  orientation_control = k_po * theta_error;

  if(pow(x_error, 2.0) + pow(y_error, 2.0) < 0.01) {
    forward_control = 0.0;
    orientation_control = 0.0;
  }
 }

int main(int argc, char** argv) {
  ros::init(argc, argv, "Obstacle_detector");
  ros::NodeHandle nh;

  ros::Publisher control_pub = nh.advertise<geometry_msgs::Twist>("hello", 10);
  ros::Subscriber pose_sub = nh.subscribe("/hello", 10, &poseCallback);

  nh.param("goal_x", x_d, 7.0);
  nh.param("goal_y", y_d, 7.0);
  nh.param("Surge_gain", k_pf, 1.0);
  nh.param("Orientiation_gain", k_po, 1.0);

  ros::Rate rate(100.0);

  while (ros::ok())
  {
    ros::spinOnce();

    geometry_msgs::Twist controls;
    controls.linear.x = forward_control;
    controls.angular.z = orientation_control;

    control_pub.publish(controls);

    rate.sleep();
  }

  return 0;
}



