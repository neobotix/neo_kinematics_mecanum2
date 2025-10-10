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

#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <neo_msgs2/msg/kinematics_state.hpp>

#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "../../common/include/Kinematics.h"
#include "../../common/include/MecanumKinematics.h"

using std::placeholders::_1;

class NeoMecanumNode : public rclcpp::Node
{
public:
  NeoMecanumNode()
  : Node("neo_mecanum_node")
  {
    topicPub_Odometry = this->create_publisher<nav_msgs::msg::Odometry>("odom", 1000);
    topicPub_DriveCommands = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "drives/joint_trajectory", 1000);
    topicSub_ComVel = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel",
      1,
      std::bind(&NeoMecanumNode::receiveCmd, this, _1));
    topicSub_DriveState = this->create_subscription<sensor_msgs::msg::JointState>(
      "drives/joint_states",
      10,
      std::bind(&NeoMecanumNode::sendOdom, this, _1));
    topicPub_KinematicsState = this->create_publisher<neo_msgs2::msg::KinematicsState>(
      "kinematics_state",
      1);
    odom_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Declaring parameters
    this->declare_parameter<double>("wheelDiameter", 0.3);
    this->declare_parameter<double>("robotWidth", 0.5);
    this->declare_parameter<double>("robotLength", 0.5);
    this->declare_parameter<double>("devX", 0.1);
    this->declare_parameter<double>("devY", 0.1);
    this->declare_parameter<double>("devZ", 0.1);
    this->declare_parameter<double>("devRoll", 0.1);
    this->declare_parameter<double>("devPitch", 0.1);
    this->declare_parameter<double>("devYaw", 0.1);
    this->declare_parameter<bool>("sendTransform", false);
    this->declare_parameter<bool>("oldHw", false);

    // Reading parameters
    this->get_parameter("wheelDiameter", wheelDiameter);
    this->get_parameter("robotWidth", axisWidth);
    this->get_parameter("robotLength", axisLength);
    this->get_parameter("devX", devX);
    this->get_parameter("devY", devY);
    this->get_parameter("devZ", devZ);
    this->get_parameter("devRoll", devRoll);
    this->get_parameter("devPitch", devPitch);
    this->get_parameter("devYaw", devYaw);
    this->get_parameter("sendTransform", sendTransform);
    this->get_parameter("oldHw", depreceatedHw);
  }

  int init()
  {
    // Creating instance of the base class
    kin = new Mecanum4WKinematics();

    // Setting the kinematics parameters to the base class
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
    last_twist.linear.x = twist->linear.x;
    last_twist.linear.y = twist->linear.y;
    last_twist.angular.z = twist->angular.z;
  }

  void sendOdom(const sensor_msgs::msg::JointState::SharedPtr js)
  {
    std::mutex m_node_mutex;

    // check if js has data from 4 motors
    if (js->velocity.size() < 4) {
      return;
    }

    // Publishing odometry message
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = js->header.stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    kin->execForwKin(js, odom, pose);
    topicPub_Odometry->publish(odom);

    // odometry transform:
    if (sendTransform) {
      geometry_msgs::msg::TransformStamped odom_trans;
      std::string robot_namespace(this->get_namespace());
      odom_trans.header.stamp = js->header.stamp;
      if (robot_namespace != "/") {
        odom_trans.header.frame_id = robot_namespace + "odom";
        odom_trans.child_frame_id = robot_namespace + "base_link";
      } else {
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";
      }
      odom_trans.transform.translation.x = odom.pose.pose.position.x;
      odom_trans.transform.translation.y = odom.pose.pose.position.y;
      odom_trans.transform.translation.z = odom.pose.pose.position.z;
      odom_trans.transform.rotation = odom.pose.pose.orientation;
      odom_broadcaster->sendTransform(odom_trans);
    }
    if (!depreceatedHw) {
      // setting the kinematic state
      kinematicsState.is_moving = false;

      kinematicsState.is_vel_cmd = false;
      if (last_twist.linear.x != 0 ||
        last_twist.linear.y != 0 ||
        last_twist.angular.z != 0)
      {
        kinematicsState.is_vel_cmd = true;
      }

    if (std::abs(odom.twist.twist.linear.x) >= 1e-3 ||
        std::abs(odom.twist.twist.linear.y) >= 1e-3 ||
        std::abs(odom.twist.twist.angular.z) >= 1e-3)
      {
        kinematicsState.is_moving = true;
      }
      topicPub_KinematicsState->publish(kinematicsState);
    }
  }

private:
  std::mutex m_node_mutex;
  Mecanum4WKinematics * kin = 0;
  bool sendTransform = false;
  bool depreceatedHw = false;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr topicPub_Odometry;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr topicPub_DriveCommands;
  rclcpp::Publisher<neo_msgs2::msg::KinematicsState>::SharedPtr topicPub_KinematicsState;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr topicSub_ComVel;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topicSub_DriveState;
  std::shared_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster;
  geometry_msgs::msg::Twist last_twist;

  double wheelDiameter, axisWidth, axisLength;
  double devX, devY, devZ, devRoll, devPitch, devYaw;
  OdomPose pose;

  neo_msgs2::msg::KinematicsState kinematicsState;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<NeoMecanumNode>();
  if (nh->init() != 0) {
    RCLCPP_ERROR_STREAM(nh->get_logger(), "neo_kinematics_mecanum_node: init failed!");
  }

  rclcpp::spin(nh);
  return 0;
}
