#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"

void CvDrive(const std_msgs::Int32 msg)
{
  ROS_INFO("I heard: %d", msg.data);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("camdata", 1000, CvDrive);

  ros::spin();

  return 0;
}
