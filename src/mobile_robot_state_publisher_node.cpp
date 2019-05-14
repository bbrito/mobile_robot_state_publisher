/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2018 \n
 *   TU Delft
 *
 *****************************************************************
 *
 * \note
 *   Project name:
 * \note
 *   ROS stack name:
 * \note
 *   ROS package name: mobile_robot_state_publisher
 *
 * \author
 *   Author: Bruno Brito, email: Bruno.deBrito@tudelft.nl
 *
 * \date Date of creation: May, 2018
 *
 * \brief
 *   This package provides a generic mobile_robot_stsate_publisher
 *
 ****************************************************************/

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/SetLinkState.h>
#include <gazebo_msgs/LinkStates.h>
#include <tf/transform_broadcaster.h>

using namespace std;

geometry_msgs::Pose pos;
geometry_msgs::Twist vel;
nav_msgs::Odometry odom_msg;


void VelocityCallBack(const gazebo_msgs::LinkStates& msg){

    pos = msg.pose[3];
    odom_msg.twist.twist = msg.twist[3];
    odom_msg.pose.pose = msg.pose[3];
    odom_msg.child_frame_id="base_link";
    odom_msg.header.frame_id="odom";
    odom_msg.header.stamp=ros::Time::now();
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin( tf::Vector3(pos.position.x,pos.position.y,pos.position.z) );
	tf::Quaternion q(pos.orientation.x,pos.orientation.y,pos.orientation.z,pos.orientation.w);

	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "mobile_robot_state_publisher_node");
	ros::NodeHandle n;
	ros::Subscriber robot_state_sub_;


	double node_rate;
	if (!n.getParam(ros::this_node::getName()+"/rate", node_rate))
	{
		ROS_ERROR_STREAM("mobile_robot_state_publisher_node Parameter " << ros::this_node::getName()+"/rate not set");
		return 0;
	}

	string root_frame;
	if (!n.getParam(ros::this_node::getName()+"/root_frame", root_frame))
	{
		ROS_ERROR_STREAM("mobile_robot_state_publisher_node Parameter " << ros::this_node::getName()+"/root_frame not set");
		return 0;
	}

	string base_frame;
	if (!n.getParam(ros::this_node::getName()+"/base_frame", base_frame))
	{
		ROS_ERROR_STREAM("mobile_robot_state_publisher_node Parameter " << ros::this_node::getName()+"/base_frame not set");
		return 0;
	}

	string robot_state_topic;
	if (!n.getParam(ros::this_node::getName()+"/robot_state_topic", robot_state_topic))
	{
		ROS_ERROR_STREAM("mobile_robot_state_publisher_node Parameter " << ros::this_node::getName()+"/robot_state_topic not set");
		return 0;
	}

	string vel_state_topic;
	if (!n.getParam(ros::this_node::getName()+"/vel_state_topic", vel_state_topic))
	{
		ROS_ERROR_STREAM("mobile_robot_state_publisher_node Parameter " << ros::this_node::getName()+"/vel_state_topic not set");
		return 0;
	}

    string subs_vel_;
    if (!n.getParam(ros::this_node::getName()+"/vel_subs", subs_vel_))
    {
       	ROS_ERROR_STREAM("mobile_robot_state_publisher_node Parameter " << ros::this_node::getName()+"/vel_subs not set");
       	return 0;
    }
	robot_state_sub_ = n.subscribe(subs_vel_, 1, VelocityCallBack);

	ros::Publisher state_pub_ =
		n.advertise<geometry_msgs::PoseStamped>(robot_state_topic, 10);
	ros::Publisher vel_pub_ =
			n.advertise<geometry_msgs::Vector3>(vel_state_topic, 10);

	ros::Rate rate(node_rate);
	geometry_msgs::PoseStamped pose_msg;
	pose_msg.header.frame_id = root_frame;
	pose_msg.header.stamp = ros::Time::now();

	geometry_msgs::Vector3 vel;
	//Intermidiate variables

	geometry_msgs::TransformStamped transformStamped;
	while (n.ok()){


		pose_msg.pose = odom_msg.pose.pose;

		vel.x = odom_msg.twist.twist.linear.x;
		vel.x = odom_msg.twist.twist.linear.y;
		state_pub_.publish(pose_msg);
		vel_pub_.publish(vel);

		ros::spinOnce();
	}

	return 0;
}
