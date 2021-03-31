/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Neobotix GmbH
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Neobotix nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include "../../common/include/Kinematics.h"
#include "../../common/include/MecanumKinematics.h"
#include "rclcpp/rclcpp.hpp"
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf2_ros/buffer.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <mutex>

using std::placeholders::_1;

class NeoMecanumNode: public rclcpp::Node 
{
public:
	NeoMecanumNode(): Node("neo_mecanum_node")
	{
	topicPub_Odometry = this->create_publisher<nav_msgs::msg::Odometry>("odom", 1000);
	topicPub_DriveCommands = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/drives/joint_trajectory", 1000);
	topicSub_ComVel = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 1, std::bind(&NeoMecanumNode::receiveCmd, this, _1));
	topicSub_DriveState = this->create_subscription<sensor_msgs::msg::JointState>("/drives/joint_states", 10, std::bind(&NeoMecanumNode::sendOdom, this, _1));
  odom_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

	this->get_parameter_or("wheelDiameter", wheelDiameter, 0.3);
	this->get_parameter_or("robotWidth", axisWidth, 0.5);
	this->get_parameter_or("robotLength", axisLength, 0.5);
	this->get_parameter_or("devX", devX, 0.1);
	this->get_parameter_or("devY", devY, 0.1);
	this->get_parameter_or("devZ", devZ, 0.1);
	this->get_parameter_or("devRoll", devRoll, 0.1);
	this->get_parameter_or("devPitch", devPitch, 0.1);
	this->get_parameter_or("devYaw", devYaw, 0.1);
	this->get_parameter_or("sendTransform", sendTransform, false);

	}

	int init()
	{
		kin = new Mecanum4WKinematics();
		kin->setWheelDiameter(wheelDiameter);
		kin->setAxis1Length(axisWidth);
		kin->setAxis2Length(axisLength);
		kin->setStdDev(devX, devY, devZ, devRoll, devPitch, devYaw);
		pose.xAbs = 0;
		pose.yAbs = 0;
		pose.phiAbs = 0;
		return 0;
	}
   	
private:
	void receiveCmd(const geometry_msgs::msg::Twist::SharedPtr twist)
	{
    std::mutex m_node_mutex;
		trajectory_msgs::msg::JointTrajectory traj;
		kin->execInvKin(twist, traj);
		topicPub_DriveCommands->publish(traj);
	}

	void sendOdom(const sensor_msgs::msg::JointState::SharedPtr js)
	{
    std::mutex m_node_mutex;
	    //check if js has data from 4 motors
		if(js->velocity.size() < 4) {
			return;
		}

		nav_msgs::msg::Odometry odom;
		odom.header.stamp = js->header.stamp;
		odom.header.frame_id = "odom";
		odom.child_frame_id = "base_link";
		kin->execForwKin(js, odom, pose);
		topicPub_Odometry->publish(odom);

		//odometry transform:
		if(sendTransform)
		{
			geometry_msgs::msg::TransformStamped odom_trans;
			odom_trans.header.stamp = odom.header.stamp;
			odom_trans.header.frame_id = odom.header.frame_id;
			odom_trans.child_frame_id = odom.child_frame_id;
			odom_trans.transform.translation.x = odom.pose.pose.position.x;
			odom_trans.transform.translation.y = odom.pose.pose.position.y;
			odom_trans.transform.translation.z = odom.pose.pose.position.z;
			odom_trans.transform.rotation = odom.pose.pose.orientation;
			odom_broadcaster->sendTransform(odom_trans);
		}
  }


private:
  std::mutex m_node_mutex;
	Mecanum4WKinematics* kin = 0;
	bool sendTransform = false;
	
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr topicPub_Odometry;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr topicPub_DriveCommands;
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr topicSub_ComVel;
	rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topicSub_DriveState;
	std::shared_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster;
	
	double wheelDiameter, axisWidth, axisLength;
	
	double devX, devY, devZ, devRoll, devPitch, devYaw;
	
	OdomPose pose;
};

int main (int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<NeoMecanumNode>();
  if(nh->init() != 0) {
    	RCLCPP_ERROR_STREAM(nh->get_logger(),"neo_kinematics_mecanum_node: init failed!");
    }
  rclcpp::spin(nh);

	return 0;
}

