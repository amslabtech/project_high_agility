#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>

sensor_msgs::Joy joy_data;
bool l1_pushed = false;

double MAX_VELOCITY;//[m/s]
double MAX_ANGULAR_VELOCITY;//[rad/s]

void joy_callback(const sensor_msgs::JoyConstPtr& msg)
{
  joy_data = *msg;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_joy");
  ros::NodeHandle nh;
  ros::NodeHandle local_nh("~");

  local_nh.getParam("/fwdis/MAX_VELOCITY", MAX_VELOCITY);
  local_nh.getParam("/fwdis/MAX_ANGULAR_VELOCITY", MAX_ANGULAR_VELOCITY);

  ros::Publisher velocity_pub = nh.advertise<geometry_msgs::Twist>("/fwdis/velocity", 1);
  ros::Publisher stop_pub = nh.advertise<std_msgs::Empty>("/stop", 1);
  ros::Publisher start_pub = nh.advertise<std_msgs::Empty>("/start", 1);

  ros::Subscriber joy_sub = nh.subscribe("/joy", 100, joy_callback);

  ros::Rate loop_rate(10);

  while(ros::ok()){
    if(!joy_data.axes.empty()){
      geometry_msgs::Twist velocity;

      if((joy_data.axes[1] == 0.0) && (joy_data.axes[0] == 0.0)){
        if(joy_data.buttons[6] || joy_data.buttons[7]){
          double omega = (joy_data.axes[5] - joy_data.axes[2]) / 2.0;
          velocity.angular.z = omega * MAX_ANGULAR_VELOCITY;
        }
      }else{
        velocity.linear.x = joy_data.axes[1] * MAX_VELOCITY;
        velocity.linear.y = joy_data.axes[0] * MAX_VELOCITY;
        double omega = (joy_data.axes[5] - joy_data.axes[2]) / 2.0;
        velocity.angular.z = omega * MAX_ANGULAR_VELOCITY;
      }
      // L1
      if(joy_data.buttons[4]){
        l1_pushed = true;
        velocity_pub.publish(velocity);
      }else{
        if(l1_pushed){
          velocity.linear.x = 0;
          velocity.linear.y = 0;
          velocity.angular.z = 0;
          velocity_pub.publish(velocity);
        }
        l1_pushed = false;
      }
      if(joy_data.buttons[1]){
        // circle
        std_msgs::Empty msg;
        stop_pub.publish(msg);
      }else if(joy_data.buttons[0]){
        // cross
        std_msgs::Empty msg;
        start_pub.publish(msg);
      }
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
