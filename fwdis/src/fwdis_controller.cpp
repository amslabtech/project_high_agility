#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <Eigen/Dense>
#include "fwdis_msgs/FourWheelDriveIndependentSteering.h"

geometry_msgs::Twist velocity;
bool velocity_subscribed = false;

double WHEEL_RADIUS;//[m]
double WHEEL_BASE;//[m]
double TREAD;//[m]
double RADIUS;
double THETA;
double INTERVAL;

Eigen::MatrixXd forward_matrix;
Eigen::VectorXd wheel_velocity;
Eigen::Vector3d robot_velocity;

void velocity_callback(const geometry_msgs::TwistConstPtr &msg)
{
  velocity = *msg;
  velocity_subscribed = true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fwdis_controller");
  ros::NodeHandle nh;

  ros::NodeHandle local_nh("~");
  local_nh.getParam("/fwdis/WHEEL_RADIUS", WHEEL_RADIUS);
  local_nh.getParam("/fwdis/WHEEL_BASE", WHEEL_BASE);
  local_nh.getParam("/fwdis/TREAD", TREAD);
  RADIUS = sqrt(pow(WHEEL_BASE, 2) + pow(TREAD, 2)) / 2.0;
  THETA = atan(TREAD / WHEEL_BASE);

  ros::Publisher command_pub = nh.advertise<fwdis_msgs::FourWheelDriveIndependentSteering>("/fwdis/command", 100);
  ros::Subscriber velocity_sub = nh.subscribe("/fwdis/velocity", 100, velocity_callback);

  forward_matrix.resize(8, 3);
  forward_matrix << 1.0, 0.0,  RADIUS * sin(THETA),
                    0.0, 1.0,  RADIUS * cos(THETA),
                    1.0, 0.0, -RADIUS * sin(THETA),
                    0.0, 1.0,  RADIUS * cos(THETA),
                    1.0, 0.0, -RADIUS * sin(THETA),
                    0.0, 1.0, -RADIUS * cos(THETA),
                    1.0, 0.0,  RADIUS * sin(THETA),
                    0.0, 1.0, -RADIUS * cos(THETA);

  wheel_velocity.resize(8, 1);

  ros::Rate loop_rate(10);

  std::cout << "fwdis_controller" << std::endl;

  while(ros::ok()){
    fwdis_msgs::FourWheelDriveIndependentSteering command;
    if(velocity_subscribed){
      robot_velocity << velocity.linear.x, velocity.linear.y, velocity.angular.z;
      wheel_velocity = forward_matrix * robot_velocity;
      command.front_right_steering_angle = atan2(wheel_velocity(1), wheel_velocity(0));
      command.front_right_wheel_velocity = sqrt(wheel_velocity(0) * wheel_velocity(0) + wheel_velocity(1) * wheel_velocity(1)) / WHEEL_RADIUS;
      command.front_left_steering_angle = atan2(wheel_velocity(3), wheel_velocity(2));
      command.front_left_wheel_velocity = sqrt(wheel_velocity(2) * wheel_velocity(2) + wheel_velocity(3) * wheel_velocity(3)) / WHEEL_RADIUS;
      command.rear_left_steering_angle = atan2(wheel_velocity(5), wheel_velocity(4));
      command.rear_left_wheel_velocity = sqrt(wheel_velocity(4) * wheel_velocity(4) + wheel_velocity(5) * wheel_velocity(5)) / WHEEL_RADIUS;
      command.rear_right_steering_angle = atan2(wheel_velocity(7), wheel_velocity(6));
      command.rear_right_wheel_velocity = sqrt(wheel_velocity(6) * wheel_velocity(6) + wheel_velocity(7) * wheel_velocity(7)) / WHEEL_RADIUS;

      if(command.front_right_steering_angle > M_PI/1.5){
        command.front_right_steering_angle -= M_PI;
        command.front_right_wheel_velocity = -command.front_right_wheel_velocity;
      }else if(command.front_right_steering_angle < -M_PI/1.5){
        command.front_right_steering_angle += M_PI;
        command.front_right_wheel_velocity = -command.front_right_wheel_velocity;
      }
      if(command.front_left_steering_angle > M_PI/1.5){
        command.front_left_steering_angle -= M_PI;
        command.front_left_wheel_velocity = -command.front_left_wheel_velocity;
      }else if(command.front_left_steering_angle < -M_PI/1.5){
        command.front_left_steering_angle += M_PI;
        command.front_left_wheel_velocity = -command.front_left_wheel_velocity;
      }
      if(command.rear_right_steering_angle > M_PI/1.5){
        command.rear_right_steering_angle -= M_PI;
        command.rear_right_wheel_velocity = -command.rear_right_wheel_velocity;
      }else if(command.rear_right_steering_angle < -M_PI/1.5){
        command.rear_right_steering_angle += M_PI;
        command.rear_right_wheel_velocity = -command.rear_right_wheel_velocity;
      }
      if(command.rear_left_steering_angle > M_PI/1.5){
        command.rear_left_steering_angle -= M_PI;
        command.rear_left_wheel_velocity = -command.rear_left_wheel_velocity;
      }else if(command.rear_left_steering_angle < -M_PI/1.5){
        command.rear_left_steering_angle += M_PI;
        command.rear_left_wheel_velocity = -command.rear_left_wheel_velocity;
      }

      command_pub.publish(command);

    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
