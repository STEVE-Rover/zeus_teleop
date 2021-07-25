#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "zero_twist_publisher");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  float rate;
  private_nh.param<float>("rate", rate, 10);
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("zero_twist", 1);
  ros::Rate r(rate);
  while (ros::ok())
  {
      pub.publish(geometry_msgs::Twist());
      r.sleep();
  }
  return 0;
}