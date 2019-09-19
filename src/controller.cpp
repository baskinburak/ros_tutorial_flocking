#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <turtlesim/Pose.h>
#include <cmath>

using Vector2 = Eigen::Matrix<double, 2, 1>;

class pd_controller {
public:
  double kp;
  double kd;
  double target;
  double prev_error;

  pd_controller(double p, double d): kp(p), kd(d) {

  }

  void change_target(double t) {
    target = t;
    prev_error = 0;
  }

  double compute_command(double current) {
    double error = target - current;

    if(error > M_PI) {
      error = error - 2 * M_PI;
    } else if(error < -M_PI) {
      error = 2 * M_PI + error;
    }

    double command = kp * error + kd * (error - prev_error);
    prev_error = error;
    return command;
  }
};

pd_controller orientation_controller(3, 1);
Vector2 desired_vel;
double current_orientation;

void desired_vel_callback(const geometry_msgs::Twist::ConstPtr& msg) {
  desired_vel(0) = msg->linear.x;
  desired_vel(1) = msg->linear.y;


  double desired_orientation = std::atan2(desired_vel(1), desired_vel(0));
  orientation_controller.change_target(desired_orientation);
}

void pose_callback(const turtlesim::Pose::ConstPtr& msg) {
  current_orientation = msg->theta;
}



int main(int argc, char* argv[]) {
  ros::init(argc, argv, "controller");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("desired_vel", 1, desired_vel_callback);
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  ros::Subscriber pose_sub = nh.subscribe("pose", 1, pose_callback);

  ros::Rate rate(10);


  while(ros::ok()) {
    ros::spinOnce();

    double command = orientation_controller.compute_command(current_orientation);

    geometry_msgs::Twist msg;
    msg.linear.x = desired_vel.norm();
    msg.linear.y = 0;
    msg.linear.z = 0;
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = command;

    pub.publish(msg);

    rate.sleep();
  }

  return 0;
}