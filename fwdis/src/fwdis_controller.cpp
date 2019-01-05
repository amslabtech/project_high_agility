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
double MAX_STEERING_ANGLE;// [rad]
double MAX_VELOCITY;// [m/s]
double MAX_ACCELERATION;// [m/ss]
double MAX_ANGULAR_VELOCITY;// [rad/s]
double MAX_ANGULAR_ACCELERATION;// [rad/ss]

const double INTERVAL = 0.1;// [s]

Eigen::MatrixXd forward_matrix;
Eigen::VectorXd wheel_velocity;
Eigen::Vector3d robot_velocity;

void velocity_callback(const geometry_msgs::TwistConstPtr &msg)
{
  geometry_msgs::Twist _velocity;
  _velocity = *msg;
  double acc_x = (_velocity.linear.x - velocity.linear.x) / INTERVAL;
  double acc_y = (_velocity.linear.y - velocity.linear.y) / INTERVAL;
  double acc_z = (_velocity.angular.z - velocity.angular.z) / INTERVAL;
  if(acc_x > MAX_ACCELERATION){
    _velocity.linear.x = velocity.linear.x + acc_x * INTERVAL;
  }else if(acc_x < -MAX_ACCELERATION){
    _velocity.linear.x = velocity.linear.x + acc_x * INTERVAL;
  }
  if(acc_y > MAX_ACCELERATION){
    _velocity.linear.y = velocity.linear.y + acc_y * INTERVAL;
  }else if(acc_z < -MAX_ACCELERATION){
    _velocity.linear.y = velocity.linear.y + acc_y * INTERVAL;
  }
  if(acc_z > MAX_ANGULAR_ACCELERATION){
    _velocity.angular.z = velocity.angular.z + acc_z * INTERVAL;
  }else if(acc_z < -MAX_ANGULAR_ACCELERATION){
    _velocity.angular.z = velocity.angular.z + acc_z * INTERVAL;
  }

  if(_velocity.linear.x > MAX_VELOCITY){
    _velocity.linear.x = MAX_VELOCITY;
  }else if(_velocity.linear.x < -MAX_VELOCITY){
    _velocity.linear.x = -MAX_VELOCITY;
  }
  if(_velocity.linear.y > MAX_VELOCITY){
    _velocity.linear.y = MAX_VELOCITY;
  }else if(_velocity.linear.y < -MAX_VELOCITY){
    _velocity.linear.y = -MAX_VELOCITY;
  }
  if(_velocity.angular.z > MAX_ANGULAR_VELOCITY){
    _velocity.angular.z = MAX_ANGULAR_VELOCITY;
  }else if(_velocity.angular.z < -MAX_ANGULAR_VELOCITY){
    _velocity.angular.z = -MAX_ANGULAR_VELOCITY;
  }
  velocity = _velocity;
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
  local_nh.getParam("/fwdis/MAX_STEERING_ANGLE", MAX_STEERING_ANGLE);
  local_nh.getParam("/fwdis/MAX_VELOCITY", MAX_VELOCITY);
  local_nh.getParam("/fwdis/MAX_ACCELERATION", MAX_ACCELERATION);
  local_nh.getParam("/fwdis/MAX_ANGULAR_VELOCITY", MAX_ANGULAR_VELOCITY);
  local_nh.getParam("/fwdis/MAX_ANGULAR_ACCELERATION", MAX_ANGULAR_ACCELERATION);
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

  ros::Rate loop_rate(1/INTERVAL);

  std::cout << "fwdis_controller" << std::endl;

  std::cout << forward_matrix << std::endl;
  std::cout << WHEEL_RADIUS << std::endl;
  std::cout << WHEEL_BASE << std::endl;
  std::cout << TREAD << std::endl;
  std::cout << MAX_STEERING_ANGLE << std::endl;
  std::cout << MAX_VELOCITY << std::endl;
  std::cout << MAX_ACCELERATION << std::endl;
  std::cout << MAX_ANGULAR_VELOCITY << std::endl;
  std::cout << MAX_ANGULAR_ACCELERATION << std::endl;

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

      if(command.front_right_steering_angle > MAX_STEERING_ANGLE){
        command.front_right_steering_angle -= M_PI;
        command.front_right_wheel_velocity = -command.front_right_wheel_velocity;
      }else if(command.front_right_steering_angle < -MAX_STEERING_ANGLE){
        command.front_right_steering_angle += M_PI;
        command.front_right_wheel_velocity = -command.front_right_wheel_velocity;
      }
      if(command.front_left_steering_angle > MAX_STEERING_ANGLE){
        command.front_left_steering_angle -= M_PI;
        command.front_left_wheel_velocity = -command.front_left_wheel_velocity;
      }else if(command.front_left_steering_angle < -MAX_STEERING_ANGLE){
        command.front_left_steering_angle += M_PI;
        command.front_left_wheel_velocity = -command.front_left_wheel_velocity;
      }
      if(command.rear_right_steering_angle > MAX_STEERING_ANGLE){
        command.rear_right_steering_angle -= M_PI;
        command.rear_right_wheel_velocity = -command.rear_right_wheel_velocity;
      }else if(command.rear_right_steering_angle < -MAX_STEERING_ANGLE){
        command.rear_right_steering_angle += M_PI;
        command.rear_right_wheel_velocity = -command.rear_right_wheel_velocity;
      }
      if(command.rear_left_steering_angle > MAX_STEERING_ANGLE){
        command.rear_left_steering_angle -= M_PI;
        command.rear_left_wheel_velocity = -command.rear_left_wheel_velocity;
      }else if(command.rear_left_steering_angle < -MAX_STEERING_ANGLE){
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
