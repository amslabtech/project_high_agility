#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <Eigen/Dense>
#include "fwdis_msgs/FourWheelDriveIndependentSteering.h"

double WHEEL_RADIUS;
double WHEEL_BASE;
double TREAD;
double RADIUS;
double THETA;
std::string ROBOT_FRAME;
const double INTERVAL = 0.01;

Eigen::MatrixXd forward_matrix;
Eigen::MatrixXd inversed_matrix;
Eigen::VectorXd wheel_velocity;//V1x, V1y, ...
Eigen::Vector3d robot_velocity;//Vx, Vy, w

class FWDISOdom
{
public:
  FWDISOdom(void);
  void drive_callback(const fwdis_msgs::FourWheelDriveIndependentSteeringConstPtr&);
  void steer_callback(const fwdis_msgs::FourWheelDriveIndependentSteeringConstPtr&);
  void process(void);

private:
  ros::NodeHandle nh;
  ros::Publisher odom_pub;
  ros::Subscriber drive_sub;
  ros::Subscriber steer_sub;
  tf::TransformBroadcaster br;
  fwdis_msgs::FourWheelDriveIndependentSteering fwdis_velocity;
  fwdis_msgs::FourWheelDriveIndependentSteering odom_drive;
  fwdis_msgs::FourWheelDriveIndependentSteering odom_steer;
  nav_msgs::Odometry odometry;
  geometry_msgs::TransformStamped odom_tf;
  tf::TransformBroadcaster odom_broadcaster;
  bool drive_updated;
  bool steer_updated;
};

//https://robotics.naist.jp/edu/text/?Robotics%2FEigen#b3b26d13
template <typename t_matrix>
t_matrix PseudoInverse(const t_matrix& m, const double &tolerance=1.e-6)
{
  using namespace Eigen;
  typedef JacobiSVD<t_matrix> TSVD;
  unsigned int svd_opt(ComputeThinU | ComputeThinV);
  if(m.RowsAtCompileTime!=Dynamic || m.ColsAtCompileTime!=Dynamic)
  svd_opt= ComputeFullU | ComputeFullV;
  TSVD svd(m, svd_opt);
  const typename TSVD::SingularValuesType &sigma(svd.singularValues());
  typename TSVD::SingularValuesType sigma_inv(sigma.size());
  for(long i=0; i<sigma.size(); ++i)
  {
    if(sigma(i) > tolerance)
      sigma_inv(i)= 1.0/sigma(i);
    else
      sigma_inv(i)= 0.0;
  }
  return svd.matrixV()*sigma_inv.asDiagonal()*svd.matrixU().transpose();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fwdis_odom");
  ros::NodeHandle nh;

  ros::NodeHandle local_nh("~");
  local_nh.getParam("/fwdis/WHEEL_RADIUS", WHEEL_RADIUS);
  local_nh.getParam("/fwdis/WHEEL_BASE", WHEEL_BASE);
  local_nh.getParam("/fwdis/TREAD", TREAD);
  local_nh.getParam("/fwdis/ROBOT_FRAME", ROBOT_FRAME);
  RADIUS = sqrt(pow(WHEEL_BASE, 2) + pow(TREAD, 2)) / 2.0;
  THETA = atan(TREAD / WHEEL_BASE);

  FWDISOdom fwdis_odom;
  fwdis_odom.process();
}

FWDISOdom::FWDISOdom(void)
{
  odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 1);
  drive_sub = nh.subscribe("/odom/drive", 1, &FWDISOdom::drive_callback, this);
  steer_sub = nh.subscribe("/odom/steer", 1, &FWDISOdom::steer_callback, this);
}

void FWDISOdom::process(void)
{

  odometry.header.frame_id = "odom";
  odometry.child_frame_id = ROBOT_FRAME;
  odometry.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);
  odom_tf.header = odometry.header;
  odom_tf.child_frame_id = odometry.child_frame_id;

  std::cout << "initialize matrix" << std::endl;

  forward_matrix.resize(8, 3);
  forward_matrix << 1.0, 0.0,  RADIUS * cos(THETA),
                    0.0, 1.0,  RADIUS * sin(THETA),
                    1.0, 0.0, -RADIUS * cos(THETA),
                    0.0, 1.0,  RADIUS * sin(THETA),
                    1.0, 0.0, -RADIUS * cos(THETA),
                    0.0, 1.0, -RADIUS * sin(THETA),
                    1.0, 0.0,  RADIUS * cos(THETA),
                    0.0, 1.0, -RADIUS * sin(THETA);


  std::cout << "forward:" << std::endl;
  std::cout << forward_matrix << std::endl;

  inversed_matrix.resize(3, 8);
  inversed_matrix = PseudoInverse(forward_matrix);

  wheel_velocity.resize(8, 1);

  std::cout << "inversed:" << std::endl;
  std::cout << inversed_matrix << std::endl;

  ros::Rate loop_rate(1.0 / INTERVAL);

  std::cout << "fwdis_odom" << std::endl;

  while(ros::ok()){
    if(drive_updated && steer_updated){
      drive_updated = false;
      steer_updated = false;
      double vfr = odom_drive.front_right_wheel_velocity * WHEEL_RADIUS;
      double vfl = odom_drive.front_left_wheel_velocity * WHEEL_RADIUS;
      double vrr = odom_drive.rear_right_wheel_velocity * WHEEL_RADIUS;
      double vrl = odom_drive.rear_left_wheel_velocity * WHEEL_RADIUS;

      wheel_velocity << vfr * cos(odom_steer.front_right_steering_angle),
                        vfr * sin(odom_steer.front_right_steering_angle),
                        vfl * cos(odom_steer.front_left_steering_angle),
                        vfl * sin(odom_steer.front_left_steering_angle),
                        vrl * cos(odom_steer.rear_left_steering_angle),
                        vrl * sin(odom_steer.rear_left_steering_angle),
                        vrr * cos(odom_steer.rear_right_steering_angle),
                        vrr * sin(odom_steer.rear_right_steering_angle);

      robot_velocity = inversed_matrix * wheel_velocity;

      double omega = robot_velocity(2);
      double vx = robot_velocity(0);
      double vy = robot_velocity(1);
      std::cout << "(vx, vy, w) = " << vx << ", " << vy << ", " << omega << std::endl;

      double yaw = tf::getYaw(odometry.pose.pose.orientation);
      odometry.pose.pose.position.x += (vx * cos(yaw) - vy * sin(yaw)) * INTERVAL;
      odometry.pose.pose.position.y += (vx * sin(yaw) + vy * cos(yaw)) * INTERVAL;
      odometry.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw + omega * INTERVAL);
      odometry.twist.twist.linear.x = vx;
      odometry.twist.twist.linear.y = vy;
      odometry.twist.twist.angular.z = omega;
    }
    odometry.header.stamp = ros::Time::now();
    odom_pub.publish(odometry);
    odom_tf.transform.translation.x = odometry.pose.pose.position.x;
    odom_tf.transform.translation.y = odometry.pose.pose.position.y;
    odom_tf.transform.rotation = odometry.pose.pose.orientation;
    odom_broadcaster.sendTransform(odom_tf);

    ros::spinOnce();
    loop_rate.sleep();

  }
}

void FWDISOdom::drive_callback(const fwdis_msgs::FourWheelDriveIndependentSteeringConstPtr& msg)
{
  odom_drive = *msg;
  drive_updated = true;
}

void FWDISOdom::steer_callback(const fwdis_msgs::FourWheelDriveIndependentSteeringConstPtr& msg)
{
  odom_steer = *msg;
  steer_updated = true;
}
