#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Empty.h>

const double INTERVAL = 0.01;

std::string ROBOT_FRAME;
std::string WORLD_FRAME;

class Stopper
{
public:
  Stopper(void);
  void waypoint_callback(const geometry_msgs::PoseArrayConstPtr&);
  void process(void);

private:
  ros::NodeHandle nh;
  ros::Publisher stop_pub;
  ros::Subscriber waypoint_sub;
  geometry_msgs::PoseArray waypoints;
  bool waypoints_subscribed;
  tf::TransformListener listener;
  tf::StampedTransform _transform;
  geometry_msgs::TransformStamped transform;
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "fwdis_stopeer");
  ros::NodeHandle nh;
  ros::NodeHandle local_nh("~");
  local_nh.getParam("/dynamic_avoidance/ROBOT_FRAME", ROBOT_FRAME);
  local_nh.getParam("/dynamic_avoidance/WORLD_FRAME", WORLD_FRAME);

  Stopper stopper;
  stopper.process();
}

Stopper::Stopper(void)
{
  stop_pub = nh.advertise<std_msgs::Empty>("/stop", 1);
  waypoint_sub = nh.subscribe("/waypoints", 1, &Stopper::waypoint_callback, this);
  waypoints_subscribed = false;
}

void Stopper::process(void)
{
  ros::Rate loop_rate(1.0 / INTERVAL);

  std::cout << "fwdis_stopper" << std::endl;

  while(ros::ok()){
    if(waypoints_subscribed){
      try{
        listener.lookupTransform(WORLD_FRAME, ROBOT_FRAME, ros::Time(0), _transform);
        tf::transformStampedTFToMsg(_transform, transform);
        double dx = waypoints.poses[1].position.x - transform.transform.translation.x;
        double dy = waypoints.poses[1].position.y - transform.transform.translation.y;
        double distance = sqrt(dx * dx + dy * dy);
        if(distance < 0.3){
          std_msgs::Empty stop_signal;
          stop_pub.publish(stop_signal);
        }
      }catch(tf::TransformException ex){
        std::cout << ex.what() << std::endl;
      }
    }

    ros::spinOnce();
    loop_rate.sleep();

  }
}

void Stopper::waypoint_callback(const geometry_msgs::PoseArrayConstPtr& msg)
{
  waypoints = *msg;
  waypoints_subscribed = true;
}
