#ifndef TELEOP_GAMPEAD_H
#define TELEOP_GAMPEAD_H

#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include <steve_teleop/VelocityTargets.h>

class TeleopGamepad
{
    public:
        TeleopGamepad(ros::NodeHandle nh, ros::NodeHandle private_nh);

    private:
        void joyCB(const sensor_msgs::Joy::ConstPtr& joy_msg);

        // ROS variables
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;
        ros::Subscriber sub_joy_;
        ros::Publisher pub_cmd_vel_;
	ros::Publisher pub_cmd_vel_tank_;

        // Parameters
        int linear_vel_axis_;
        int angular_vel_axis_;
	int left_vel_axis_;
	int right_vel_axis_;
	int slow_vel_axis_;
        int deadman_switch_;
	int deadman_switch_tank_;
        int turbo_button_;
        float max_linear_vel_;
        float max_angular_vel_;
        float turbo_max_linear_vel_;
        float turbo_max_angular_vel_;
};


#endif
