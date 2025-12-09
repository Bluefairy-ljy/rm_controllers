/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, Qiayuan Liao
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

//
// Created by huakang on 2021/3/21.
//
#include "rm_chassis_controllers/chassis_base.h"
#include <rm_common/ros_utilities.h>
#include <rm_common/math_utilities.h>
#include <rm_common/ori_tool.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <angles/angles.h>
#include <hardware_interface/imu_sensor_interface.h>

namespace rm_chassis_controllers
{
template class ChassisBase<rm_control::RobotStateInterface, hardware_interface::EffortJointInterface>;
template class ChassisBase<rm_control::RobotStateInterface, hardware_interface::ImuSensorInterface,
                           hardware_interface::EffortJointInterface>;

template <typename... T>
void ChassisBase<T...>::initialize_parameters(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh,
                                              ros::NodeHandle& controller_nh)
{
  try
  {
    controller_nh.getParam("publish_rate", publish_rate_);
    controller_nh.getParam("publish_map_tf", publish_map_tf_);
    controller_nh.getParam("publish_odom_tf", publish_odom_tf_);
    controller_nh.getParam("enable_wheel_odom", enable_wheel_odom_);
    controller_nh.getParam("enable_slam_odom", enable_slam_odom_);
    controller_nh.getParam("global_map_frame_id", global_map_frame_id_);
    controller_nh.getParam("imu_odom_frame_id", imu_odom_frame_id_);
    controller_nh.getParam("robot_odom_frame_id", robot_odom_frame_id_);
    controller_nh.getParam("robot_base_frame_id", robot_base_frame_id_);
    controller_nh.getParam("lidar_odom_frame_id", lidar_odom_frame_id_);
    controller_nh.getParam("lidar_base_frame_id", lidar_base_frame_id_);
    controller_nh.getParam("slam_odom_topic", slam_odom_topic_);
    controller_nh.getParam("localization_result_topic", localization_result_topic_);

    controller_nh.getParam("timeout", timeout_);
    controller_nh.getParam("power/effort_coeff", effort_coeff_);
    controller_nh.getParam("power/vel_coeff", velocity_coeff_);
    controller_nh.getParam("power/power_offset", power_offset_);
    controller_nh.getParam("wheel_radius", wheel_radius_);
    controller_nh.getParam("twist_angular", twist_angular_);
    controller_nh.getParam("max_odom_vel", max_odom_vel_);

    if (controller_nh.hasParam("pid_follow"))
      pid_follow_.init(ros::NodeHandle(controller_nh, "pid_follow"));

    ramp_x_ = new RampFilter<double>(0, 0.001);
    ramp_y_ = new RampFilter<double>(0, 0.001);
    ramp_w_ = new RampFilter<double>(0, 0.001);

    // Setup nav_msgs::Odometry realtime publisher for move_base_flex
    nav_odometry_rtpub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(root_nh, "odom", 100));
    nav_odometry_rtpub_->msg_.header.frame_id = global_map_frame_id_;
    nav_odometry_rtpub_->msg_.child_frame_id = robot_base_frame_id_;
    nav_odometry_rtpub_->msg_.pose.pose.position.x = 0.0;
    nav_odometry_rtpub_->msg_.pose.pose.position.y = 0.0;
    nav_odometry_rtpub_->msg_.pose.pose.position.z = 0.0;
    nav_odometry_rtpub_->msg_.pose.pose.orientation.x = 0.0;
    nav_odometry_rtpub_->msg_.pose.pose.orientation.y = 0.0;
    nav_odometry_rtpub_->msg_.pose.pose.orientation.z = 0.0;
    nav_odometry_rtpub_->msg_.pose.pose.orientation.w = 1.0;
    nav_odometry_rtpub_->msg_.twist.twist.linear.x = 0.0;
    nav_odometry_rtpub_->msg_.twist.twist.linear.y = 0.0;
    nav_odometry_rtpub_->msg_.twist.twist.linear.z = 0.0;
    nav_odometry_rtpub_->msg_.twist.twist.angular.x = 0.0;
    nav_odometry_rtpub_->msg_.twist.twist.angular.y = 0.0;
    nav_odometry_rtpub_->msg_.twist.twist.angular.z = 0.0;
    constexpr double twist_cov = 0.001;
    // diagonal[twist_cov]
    nav_odometry_rtpub_->msg_.twist.covariance = { twist_cov, 0., 0., 0., 0., 0., 0., twist_cov, 0., 0., 0., 0., 0., 0.,
                                                   twist_cov, 0., 0., 0., 0., 0., 0., twist_cov, 0., 0., 0., 0., 0., 0.,
                                                   twist_cov, 0., 0., 0., 0., 0., 0., twist_cov };

    last_debug_time_wheel_ = ros::Time::now();
    last_debug_time_slam_ = ros::Time::now();
    last_debug_time_imu_ = ros::Time::now();

    robot_state_handle_ = robot_hw->get<rm_control::RobotStateInterface>()->getHandle("robot_state");
    effort_joint_interface_ = robot_hw->get<hardware_interface::EffortJointInterface>();

    // slam_odom_sub_ =
    //     controller_nh.subscribe<nav_msgs::Path>(slam_odom_topic_, 10, &ChassisBase::slamOdomCallback, this);
    slam_odom_sub_ =
        controller_nh.subscribe<nav_msgs::Odometry>(slam_odom_topic_, 10, &ChassisBase::slamOdomCallback, this);
    localization_result_sub_ = controller_nh.subscribe<geometry_msgs::TransformStamped>(
        localization_result_topic_, 10, &ChassisBase::localizationResultCallback, this);
    cmd_chassis_sub_ =
        controller_nh.subscribe<rm_msgs::ChassisCmd>("/cmd_chassis", 1, &ChassisBase::cmdChassisCallback, this);
    cmd_vel_sub_ = root_nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &ChassisBase::cmdVelCallback, this);
  }
  catch (...)
  {
    ROS_ERROR("Some chassis params doesn't given (namespace: %s)", controller_nh.getNamespace().c_str());
  }
}

template <typename... T>
bool ChassisBase<T...>::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh,
                             ros::NodeHandle& controller_nh)
{
  initialize_parameters(robot_hw, root_nh, controller_nh);

  if (enable_wheel_odom_)
  {  // init wheel_odom
    wheel_odometry_.header.stamp = ros::Time::now();
    wheel_odometry_.header.frame_id = robot_odom_frame_id_;
    wheel_odometry_.child_frame_id = robot_base_frame_id_;
    wheel_odometry_.pose.pose.position.x = 0.0;
    wheel_odometry_.pose.pose.position.y = 0.0;
    wheel_odometry_.pose.pose.position.z = 0.0;
    wheel_odometry_.pose.pose.orientation.x = 0.0;
    wheel_odometry_.pose.pose.orientation.y = 0.0;
    wheel_odometry_.pose.pose.orientation.z = 0.0;
    wheel_odometry_.pose.pose.orientation.w = 1.0;
    wheel_odometry_.twist.twist.linear.x = 0.0;
    wheel_odometry_.twist.twist.linear.y = 0.0;
    wheel_odometry_.twist.twist.linear.z = 0.0;
    wheel_odometry_.twist.twist.angular.x = 0.0;
    wheel_odometry_.twist.twist.angular.y = 0.0;
    wheel_odometry_.twist.twist.angular.z = 0.0;
  }

  if (publish_map_tf_)
  {  // init global_map -> robot_odom tf
    try
    {
      global_map2robot_odom_.header.stamp = ros::Time::now();
      global_map2robot_odom_.header.frame_id = global_map_frame_id_;
      global_map2robot_odom_.child_frame_id = robot_odom_frame_id_;
      global_map2robot_odom_.transform.translation.x = 0;
      global_map2robot_odom_.transform.translation.y = 0;
      global_map2robot_odom_.transform.translation.z = 0;
      global_map2robot_odom_.transform.rotation.x = 0;
      global_map2robot_odom_.transform.rotation.y = 0;
      global_map2robot_odom_.transform.rotation.z = 0;
      global_map2robot_odom_.transform.rotation.w = 1;
      tf_broadcaster4global_map2robot_odom_.init(root_nh);
      tf_broadcaster4global_map2robot_odom_.sendTransform(global_map2robot_odom_);
      ROS_INFO("Initialized global_map -> robot_odom tf troadcaster");
    }
    catch (...)
    {
      ROS_ERROR("Failed to init global_map -> robot_odom tf broadcaster");
      return false;
    }
  }

  if (publish_odom_tf_)
  {  // init robot_odom -> robot_base
    try
    {
      robot_odom2imu_odom_.header.stamp = ros::Time::now();
      robot_odom2imu_odom_.header.frame_id = robot_odom_frame_id_;
      robot_odom2imu_odom_.child_frame_id = imu_odom_frame_id_;
      robot_odom2imu_odom_.transform.translation.x = 0;
      robot_odom2imu_odom_.transform.translation.y = 0;
      robot_odom2imu_odom_.transform.translation.z = 0;
      robot_odom2imu_odom_.transform.rotation.x = 0;
      robot_odom2imu_odom_.transform.rotation.y = 0;
      robot_odom2imu_odom_.transform.rotation.z = 0;
      robot_odom2imu_odom_.transform.rotation.w = 1;
      tf_broadcaster4robot_odom2imu_odom_.init(root_nh);
      tf_broadcaster4robot_odom2imu_odom_.sendTransform(robot_odom2imu_odom_);
      ROS_INFO("Initialized robot_odom -> imu_odom tf broadcaster");
    }
    catch (...)
    {
      ROS_ERROR("Failed to init robot_odom -> robot_base tf broadcaster");
      return false;
    }
  }
  if (enable_slam_odom_)
  {
    try
    {  // init robot_odom->lidar_odom tf
      // tf_broadcaster4robot_odom2lidar_odom_.init(root_nh);
      robot_odom2lidar_odom_.header.stamp = ros::Time::now();
      robot_odom2lidar_odom_.header.frame_id = robot_odom_frame_id_;
      robot_odom2lidar_odom_.child_frame_id = lidar_odom_frame_id_;
      auto trans = robot_state_handle_.lookupTransform(robot_base_frame_id_, lidar_base_frame_id_, ros::Time(0));
      robot_odom2lidar_odom_.transform.translation.x = trans.transform.translation.x;
      robot_odom2lidar_odom_.transform.translation.y = trans.transform.translation.y;
      robot_odom2lidar_odom_.transform.translation.z = trans.transform.translation.z;
      robot_odom2lidar_odom_.transform.rotation.x = trans.transform.rotation.x;
      robot_odom2lidar_odom_.transform.rotation.y = trans.transform.rotation.y;
      robot_odom2lidar_odom_.transform.rotation.z = trans.transform.rotation.z;
      robot_odom2lidar_odom_.transform.rotation.w = trans.transform.rotation.w;
      // tf_broadcaster4robot_odom2lidar_odom_.init(root_nh);
      // tf_broadcaster4robot_odom2lidar_odom_.sendTransform(robot_odom2lidar_odom_);
      // slam_odom_initialized_ = true;
      ROS_INFO("Initialized robot_odom -> lidar_odom tf");
    }
    catch (...)
    {
      ROS_ERROR("Failed to init robot_odom -> lidar_odom tf broadcaster");
      return false;
    }
  }

  return true;
}

template <typename... T>
void ChassisBase<T...>::update(const ros::Time& time, const ros::Duration& period)
{
  rm_msgs::ChassisCmd cmd_chassis = cmd_rt_buffer_.readFromRT()->cmd_chassis_;
  geometry_msgs::Twist cmd_vel = cmd_rt_buffer_.readFromRT()->cmd_vel_;

  if ((time - cmd_rt_buffer_.readFromRT()->stamp_).toSec() > timeout_)
  {
    vel_cmd_.x = 0.;
    vel_cmd_.y = 0.;
    vel_cmd_.z = 0.;
  }
  else
  {
    ramp_x_->setAcc(cmd_chassis.accel.linear.x);
    ramp_y_->setAcc(cmd_chassis.accel.linear.y);
    ramp_x_->input(cmd_vel.linear.x);
    ramp_y_->input(cmd_vel.linear.y);
    vel_cmd_.x = ramp_x_->output();
    vel_cmd_.y = ramp_y_->output();
    vel_cmd_.z = cmd_vel.angular.z;
  }

  if (cmd_rt_buffer_.readFromRT()->cmd_chassis_.follow_source_frame.empty())
    follow_source_frame_ = "yaw";
  else
    follow_source_frame_ = cmd_rt_buffer_.readFromRT()->cmd_chassis_.follow_source_frame;
  if (cmd_rt_buffer_.readFromRT()->cmd_chassis_.command_source_frame.empty())
    command_source_frame_ = "yaw";
  else
    command_source_frame_ = cmd_rt_buffer_.readFromRT()->cmd_chassis_.command_source_frame;

  if (state_ != cmd_chassis.mode)
  {
    state_ = cmd_chassis.mode;
    state_changed_ = true;
  }

  updateOdom(time, period);

  switch (state_)
  {
    case RAW:
      raw();
      break;
    case FOLLOW:
      follow(time, period);
      break;
    case TWIST:
      twist(time, period);
      break;
  }

  ramp_w_->setAcc(cmd_chassis.accel.angular.z);
  ramp_w_->input(vel_cmd_.z);
  vel_cmd_.z = ramp_w_->output();

  moveJoint(time, period);
  powerLimit();
}

template <typename... T>
void ChassisBase<T...>::follow(const ros::Time& time, const ros::Duration& period)
{
  if (state_changed_)
  {
    state_changed_ = false;
    ROS_INFO("[Chassis] Enter FOLLOW");

    recovery();
    pid_follow_.reset();
  }

  tfVelToBase(command_source_frame_);
  try
  {
    double roll{}, pitch{}, yaw{};
    quatToRPY(
        robot_state_handle_.lookupTransform(robot_base_frame_id_, follow_source_frame_, ros::Time(0)).transform.rotation,
        roll, pitch, yaw);
    double follow_error = angles::shortest_angular_distance(yaw, 0);
    pid_follow_.computeCommand(-follow_error, period);
    vel_cmd_.z = pid_follow_.getCurrentCmd() + cmd_rt_buffer_.readFromRT()->cmd_chassis_.follow_vel_des;
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
  }
}

template <typename... T>
void ChassisBase<T...>::twist(const ros::Time& time, const ros::Duration& period)
{
  if (state_changed_)
  {
    state_changed_ = false;
    ROS_INFO("[Chassis] Enter TWIST");

    recovery();
    pid_follow_.reset();
  }
  tfVelToBase(command_source_frame_);
  try
  {
    double roll{}, pitch{}, yaw{};
    quatToRPY(robot_state_handle_.lookupTransform(robot_base_frame_id_, command_source_frame_, ros::Time(0))
                  .transform.rotation,
              roll, pitch, yaw);

    double angle[4] = { -0.785, 0.785, 2.355, -2.355 };
    double off_set = 0.0;
    for (double i : angle)
    {
      if (std::abs(angles::shortest_angular_distance(yaw, i)) < 0.79)
      {
        off_set = i;
        break;
      }
    }
    double follow_error =
        angles::shortest_angular_distance(yaw, twist_angular_ * sin(2 * M_PI * time.toSec()) + off_set);

    pid_follow_.computeCommand(-follow_error, period);  // The actual output is opposite to the calculated value
    vel_cmd_.z = pid_follow_.getCurrentCmd();
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
  }
}

template <typename... T>
void ChassisBase<T...>::raw()
{
  if (state_changed_)
  {
    state_changed_ = false;
    ROS_INFO("[Chassis] Enter RAW");

    recovery();
  }
  tfVelToBase(command_source_frame_);
}

template <typename... T>
void ChassisBase<T...>::updateOdom(const ros::Time& time, const ros::Duration& period)
{
  geometry_msgs::Twist vel_base = odometry();  // wheel_odom twist on base_link frame
  geometry_msgs::Vector3 linear_vel_odom, angular_vel_odom;
  // transform vel_base to robot_odom frame
  tf2::doTransform(vel_base.linear, linear_vel_odom, robot_odom2robot_base_);
  tf2::doTransform(vel_base.angular, angular_vel_odom, robot_odom2robot_base_);

  try
  {  // integral wheel_odom to nav_odometry.twist
    auto& nav_odometry = nav_odometry_rtpub_->msg_;
    nav_odometry.header.stamp = time;
    nav_odometry.twist.twist.linear.x = linear_vel_odom.x;
    nav_odometry.twist.twist.linear.y = linear_vel_odom.y;
    nav_odometry.twist.twist.linear.z = linear_vel_odom.z;
    nav_odometry.twist.twist.angular.x = angular_vel_odom.x;
    nav_odometry.twist.twist.angular.y = angular_vel_odom.y;
    nav_odometry.twist.twist.angular.z = angular_vel_odom.z;
  }
  catch (...)
  {
    ROS_ERROR("Failed to update nav_odometry twist");
    return;
  }

  if (enable_wheel_odom_)
  {
    try
    {
      // update wheel_odom twist
      wheel_odometry_.header.stamp = time;
      wheel_odometry_.twist.twist.linear.x = linear_vel_odom.x;
      wheel_odometry_.twist.twist.linear.y = linear_vel_odom.y;
      wheel_odometry_.twist.twist.linear.z = linear_vel_odom.z;
      wheel_odometry_.twist.twist.angular.x = angular_vel_odom.x;
      wheel_odometry_.twist.twist.angular.y = angular_vel_odom.y;
      wheel_odometry_.twist.twist.angular.z = angular_vel_odom.z;
      // update wheel_odom position
      double modulus =
          std::sqrt(std::pow(linear_vel_odom.x, 2) + std::pow(linear_vel_odom.y, 2) + std::pow(linear_vel_odom.z, 2));
      if (modulus > 0.01 && modulus < max_odom_vel_)
      {  // avoid nan pos
        wheel_odometry_.pose.pose.position.x += linear_vel_odom.x * period.toSec();
        wheel_odometry_.pose.pose.position.y += linear_vel_odom.y * period.toSec();
        wheel_odometry_.pose.pose.position.z += linear_vel_odom.z * period.toSec();
      }

      // update wheel_odom orientation
      modulus = std::sqrt(std::pow(angular_vel_odom.x, 2) + std::pow(angular_vel_odom.y, 2) +
                          std::pow(angular_vel_odom.z, 2));
      if (modulus > 0.001)
      {  // avoid nan quat
        tf2::Quaternion current_quat, addon_quat;
        tf2::fromMsg(wheel_odometry_.pose.pose.orientation, current_quat);
        addon_quat.setRotation(tf2::Vector3(angular_vel_odom.x / modulus, angular_vel_odom.y / modulus,
                                            angular_vel_odom.z / modulus),
                               modulus * period.toSec());
        current_quat = addon_quat * current_quat;
        current_quat.normalize();
        wheel_odometry_.pose.pose.orientation.x = current_quat.x();
        wheel_odometry_.pose.pose.orientation.y = current_quat.y();
        wheel_odometry_.pose.pose.orientation.z = current_quat.z();
        wheel_odometry_.pose.pose.orientation.w = current_quat.w();
      }
      // if (ros::Time::now() - last_debug_time_wheel_ > ros::Duration(1.0))
      // {  // debug message
      //   ROS_INFO("wheel_odometry_ twist %lf %lf %lf", wheel_odometry_.twist.twist.linear.x,
      //            wheel_odometry_.twist.twist.linear.y, wheel_odometry_.twist.twist.angular.z);
      //   ROS_INFO("wheel_odometry_ position %lf %lf %lf", wheel_odometry_.pose.pose.position.x,
      //            wheel_odometry_.pose.pose.position.y, wheel_odometry_.pose.pose.position.z);
      //   ROS_INFO("wheel_odometry_ orient %lf %lf %lf %lf", wheel_odometry_.pose.pose.orientation.x,
      //            wheel_odometry_.pose.pose.orientation.y, wheel_odometry_.pose.pose.orientation.z,
      //            wheel_odometry_.pose.pose.orientation.w);
      //   last_debug_time_wheel_ = ros::Time::now();
      // }
    }
    catch (...)
    {
      ROS_ERROR("Failed to update wheel_odom");
      return;
    }
  }

  if (enable_slam_odom_ && topic_update_)
  {  // update tf robot_odom -> robot_base using slam_odom
    if (!slam_odom_initialized_)
    {
      try
      {  // init robot_odom->lidar_odom tf
        robot_odom2lidar_odom_.header.stamp = time;
        robot_odom2lidar_odom_.header.frame_id = robot_odom_frame_id_;
        robot_odom2lidar_odom_.child_frame_id = lidar_odom_frame_id_;
        auto trans = robot_state_handle_.lookupTransform(robot_base_frame_id_, lidar_base_frame_id_, ros::Time(0));
        robot_odom2lidar_odom_.transform.translation.x = trans.transform.translation.x;
        robot_odom2lidar_odom_.transform.translation.y = trans.transform.translation.y;
        robot_odom2lidar_odom_.transform.translation.z = trans.transform.translation.z;
        robot_odom2lidar_odom_.transform.rotation.x = trans.transform.rotation.x;
        robot_odom2lidar_odom_.transform.rotation.y = trans.transform.rotation.y;
        robot_odom2lidar_odom_.transform.rotation.z = trans.transform.rotation.z;
        robot_odom2lidar_odom_.transform.rotation.w = trans.transform.rotation.w;
        slam_odom_initialized_ = true;
        ROS_INFO("Initialized robot_odom -> lidar_odom tf");
      }
      catch (...)
      {
        ROS_ERROR("Failed to init robot_odom -> lidar_odom tf broadcaster");
        return;
      }
    }

    try
    {  // update slam_odom to lidar_odom->lidar_base
      auto* slam_msg = slam_rt_buffer_.readFromRT();
      lidar_odom2lidar_base_.transform.translation.x = slam_msg->pose.pose.position.x;
      lidar_odom2lidar_base_.transform.translation.y = slam_msg->pose.pose.position.y;
      lidar_odom2lidar_base_.transform.translation.z = slam_msg->pose.pose.position.z;
      lidar_odom2lidar_base_.transform.rotation.x = slam_msg->pose.pose.orientation.x;
      lidar_odom2lidar_base_.transform.rotation.y = slam_msg->pose.pose.orientation.y;
      lidar_odom2lidar_base_.transform.rotation.z = slam_msg->pose.pose.orientation.z;
      lidar_odom2lidar_base_.transform.rotation.w = slam_msg->pose.pose.orientation.w;
      topic_update_ = false;
      lidar_base2robot_base_ =
          robot_state_handle_.lookupTransform(lidar_base_frame_id_, robot_base_frame_id_, slam_msg->header.stamp);
    }
    catch (...)
    {
      ROS_ERROR("Failed to update slam_odom");
      return;
    }

    try
    {  // mix slam_odom to robot_odom->robot_base
      robot_base2imu_odom_ =
          robot_state_handle_.lookupTransform(robot_base_frame_id_, imu_odom_frame_id_, ros::Time(0));

      // {  // Extract yaw from rotation and discard roll/pitch
      //   double roll, pitch, yaw;
      //   quatToRPY(robot_base2imu_odom_.transform.rotation, roll, pitch, yaw);

      //   // Create new quaternion with only yaw rotation
      //   tf2::Quaternion yaw_only_quat;
      //   yaw_only_quat.setRPY(0.0, 0.0, yaw);

      //   // Assign back to robot_base2imu_odom_
      //   robot_base2imu_odom_.transform.rotation.x = yaw_only_quat.x();
      //   robot_base2imu_odom_.transform.rotation.y = yaw_only_quat.y();
      //   robot_base2imu_odom_.transform.rotation.z = yaw_only_quat.z();
      //   robot_base2imu_odom_.transform.rotation.w = yaw_only_quat.w();

      //   robot_base2imu_odom_.transform.translation.x = 0;
      //   robot_base2imu_odom_.transform.translation.y = 0;
      //   robot_base2imu_odom_.transform.translation.z = 0;
      // }

      tf2::fromMsg(robot_odom2lidar_odom_.transform, tf_robot_odom2lidar_odom_);
      tf2::fromMsg(lidar_odom2lidar_base_.transform, tf_lidar_odom2lidar_base_);
      tf2::fromMsg(lidar_base2robot_base_.transform, tf_lidar_base2robot_base_);
      tf2::fromMsg(robot_base2imu_odom_.transform, tf_robot_base2imu_odom_);

      tf_robot_odom2imu_odom_ =
          tf_robot_odom2lidar_odom_ * tf_lidar_odom2lidar_base_ * tf_lidar_base2robot_base_ * tf_robot_base2imu_odom_;
      robot_odom2imu_odom_.transform = tf2::toMsg(tf_robot_odom2imu_odom_);
      robot_state_handle_.setTransform(robot_odom2imu_odom_, "rm_chassis_controllers");
    }
    catch (...)
    {
      ROS_ERROR("Failed to update robot_odom->imu_odom");
      return;
    }

    // if (ros::Time::now() - last_debug_time_imu_ > ros::Duration(1.0))
    // {  // debug message
    //   ROS_INFO("imu_odom position %lf %lf %lf", robot_odom2imu_odom_.transform.translation.x,
    //            robot_odom2imu_odom_.transform.translation.y, robot_odom2imu_odom_.transform.translation.z);
    //   ROS_INFO("imu_odom orient %lf %lf %lf %lf", robot_odom2imu_odom_.transform.rotation.x,
    //            robot_odom2imu_odom_.transform.rotation.y, robot_odom2imu_odom_.transform.rotation.z,
    //            robot_odom2imu_odom_.transform.rotation.w);
    //   last_debug_time_imu_ = ros::Time::now();
    // }

    // if (ros::Time::now() - last_debug_time_slam_ > ros::Duration(1.0))
    // {  // debug message
    //   ROS_INFO("slam_odometry position %lf %lf %lf", lidar_odom2lidar_base_.transform.translation.x,
    //            lidar_odom2lidar_base_.transform.translation.y, lidar_odom2lidar_base_.transform.translation.z);
    //   // ROS_INFO("slam_odometry orient %lf %lf %lf %lf", lidar_odom2lidar_base_.transform.rotation.x,
    //   //          lidar_odom2lidar_base_.transform.rotation.y, lidar_odom2lidar_base_.transform.rotation.z,
    //   //          lidar_odom2lidar_base_.transform.rotation.w);
    //   last_debug_time_slam_ = ros::Time::now();
    // }

    //   if (ros::Time::now() - last_debug_time_slam_ > ros::Duration(1.0))
    //   {  // debug message
    //     ROS_INFO("robot_odometry position %lf %lf %lf", robot_odom2robot_base_.transform.translation.x,
    //              robot_odom2robot_base_.transform.translation.y, robot_odom2robot_base_.transform.translation.z);
    //     // ROS_INFO("robot_odometry orient %lf %lf %lf %lf", robot_odom2robot_base_.transform.rotation.x,
    //     //          robot_odom2robot_base_.transform.rotation.y, robot_odom2robot_base_.transform.rotation.z,
    //     //          robot_odom2robot_base_.transform.rotation.w);
    //     last_debug_time_slam_ = ros::Time::now();
    //   }
    // }

    if (ros::Time::now() - last_debug_time_slam_ > ros::Duration(1.0))
    {  // debug message
      ROS_INFO("robot_odometry position %lf %lf %lf", robot_odom2imu_odom_.transform.translation.x,
               robot_odom2imu_odom_.transform.translation.y, robot_odom2imu_odom_.transform.translation.z);
      // ROS_INFO("robot_odometry orient %lf %lf %lf %lf", robot_odom2imu_odom_.transform.rotation.x,
      //          robot_odom2imu_odom_.transform.rotation.y, robot_odom2imu_odom_.transform.rotation.z,
      //          robot_odom2imu_odom_.transform.rotation.w);
      last_debug_time_slam_ = ros::Time::now();
    }
  }

  if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0 / publish_rate_) < time)
  {  // publish nav_msgs::Odometry and tfs
    if (nav_odometry_rtpub_->trylock())
    {
      try
      {  // integral global_map -> robot_base to nav_odometry.pose
        auto& nav_odometry = nav_odometry_rtpub_->msg_;

        nav_odometry.header.stamp = time;
        global_map2robot_base_ =
            robot_state_handle_.lookupTransform(global_map_frame_id_, robot_base_frame_id_, ros::Time(0));
        nav_odometry.pose.pose.position.x = global_map2robot_base_.transform.translation.x;
        nav_odometry.pose.pose.position.y = global_map2robot_base_.transform.translation.y;
        nav_odometry.pose.pose.position.z = global_map2robot_base_.transform.translation.z;

        tf2::Quaternion map2base_quat;
        map2base_quat =
            tf2::Quaternion(global_map2robot_base_.transform.rotation.x, global_map2robot_base_.transform.rotation.y,
                            global_map2robot_base_.transform.rotation.z, global_map2robot_base_.transform.rotation.w);
        nav_odometry.pose.pose.orientation.x = map2base_quat.getX();
        nav_odometry.pose.pose.orientation.y = map2base_quat.getY();
        nav_odometry.pose.pose.orientation.z = map2base_quat.getZ();
        nav_odometry.pose.pose.orientation.w = map2base_quat.getW();

        nav_odometry_rtpub_->unlockAndPublish();
      }
      catch (...)
      {
        ROS_WARN("Failed to publish nav_odometry");
      }
    }

    if (publish_map_tf_ && publish_odom_tf_)
    {
      try
      {  // publish tfs
        global_map2robot_odom_.header.stamp = time;
        tf_broadcaster4global_map2robot_odom_.sendTransform(global_map2robot_odom_);

        robot_odom2imu_odom_.header.stamp = time;
        tf_broadcaster4robot_odom2imu_odom_.sendTransform(robot_odom2imu_odom_);

        last_publish_time_ = time;
      }
      catch (...)
      {
        ROS_WARN("Failed to publish tfs");
      }
    }
  }
}

template <typename... T>
void ChassisBase<T...>::recovery()
{
  ramp_x_->clear(vel_cmd_.x);
  ramp_y_->clear(vel_cmd_.y);
  ramp_w_->clear(vel_cmd_.z);
}

template <typename... T>
void ChassisBase<T...>::powerLimit()
{
  double power_limit = cmd_rt_buffer_.readFromRT()->cmd_chassis_.power_limit;
  // Three coefficients of a quadratic equation in one variable
  double a = 0., b = 0., c = 0.;
  for (const auto& joint : joint_handles_)
  {
    double cmd_effort = joint.getCommand();
    double real_vel = joint.getVelocity();
    if (joint.getName().find("wheel") != std::string::npos)  // The pivot joint of swerve drive doesn't need power limit
    {
      a += square(cmd_effort);
      b += std::abs(cmd_effort * real_vel);
      c += square(real_vel);
    }
  }
  a *= effort_coeff_;
  c = c * velocity_coeff_ - power_offset_ - power_limit;
  // Root formula for quadratic equation in one variable
  double zoom_coeff = (square(b) - 4 * a * c) > 0 ? ((-b + sqrt(square(b) - 4 * a * c)) / (2 * a)) : 0.;
  for (auto joint : joint_handles_)
    if (joint.getName().find("wheel") != std::string::npos)
    {
      joint.setCommand(zoom_coeff > 1 ? joint.getCommand() : joint.getCommand() * zoom_coeff);
    }
}

template <typename... T>
void ChassisBase<T...>::tfVelToBase(const std::string& from)
{
  try
  {
    tf2::doTransform(vel_cmd_, vel_cmd_, robot_state_handle_.lookupTransform(robot_base_frame_id_, from, ros::Time(0)));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
  }
}

template <typename... T>
void ChassisBase<T...>::cmdChassisCallback(const rm_msgs::ChassisCmdConstPtr& msg)
{
  cmd_struct_.cmd_chassis_ = *msg;
  cmd_rt_buffer_.writeFromNonRT(cmd_struct_);
}

template <typename... T>
void ChassisBase<T...>::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  cmd_struct_.cmd_vel_ = *msg;
  cmd_struct_.stamp_ = ros::Time::now();
  cmd_rt_buffer_.writeFromNonRT(cmd_struct_);
}

// template <typename... T>
// void ChassisBase<T...>::slamOdomCallback(const nav_msgs::Path::ConstPtr& msg)
// {
//   nav_msgs::Odometry slam_odom;
//   slam_odom.header = msg->header;
//   if (msg->poses.empty())
//     return;
//   slam_odom.pose.pose.position.x = msg->poses.back().pose.position.x;
//   slam_odom.pose.pose.position.y = msg->poses.back().pose.position.y;
//   slam_odom.pose.pose.position.z = msg->poses.back().pose.position.z;
//   slam_odom.pose.pose.orientation.x = msg->poses.back().pose.orientation.x;
//   slam_odom.pose.pose.orientation.y = msg->poses.back().pose.orientation.y;
//   slam_odom.pose.pose.orientation.z = msg->poses.back().pose.orientation.z;
//   slam_odom.pose.pose.orientation.w = msg->poses.back().pose.orientation.w;
//   slam_rt_buffer_.writeFromNonRT(slam_odom);
//   topic_update_ = true;
// }

template <typename... T>
void ChassisBase<T...>::slamOdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  slam_rt_buffer_.writeFromNonRT(*msg);
  topic_update_ = true;
}

template <typename... T>
void ChassisBase<T...>::localizationResultCallback(const geometry_msgs::TransformStamped::ConstPtr& msg)
{
  global_map2lidar_odom_.header.stamp = msg->header.stamp;
  global_map2lidar_odom_.transform.translation.x = msg->transform.translation.x;
  global_map2lidar_odom_.transform.translation.y = msg->transform.translation.y;
  global_map2lidar_odom_.transform.translation.z = msg->transform.translation.z;
  global_map2lidar_odom_.transform.rotation.x = msg->transform.rotation.x;
  global_map2lidar_odom_.transform.rotation.y = msg->transform.rotation.y;
  global_map2lidar_odom_.transform.rotation.z = msg->transform.rotation.z;
  global_map2lidar_odom_.transform.rotation.w = msg->transform.rotation.w;

  tf2::Transform tf_global_map2lidar_odom, tf_robot_odom2lidar_odom;
  tf2::fromMsg(global_map2lidar_odom_.transform, tf_global_map2lidar_odom);
  tf2::fromMsg(robot_odom2lidar_odom_.transform, tf_robot_odom2lidar_odom);

  tf2::Transform tf_global_map2robot_odom = tf_global_map2lidar_odom * tf_robot_odom2lidar_odom.inverse();
  global_map2robot_odom_.transform = tf2::toMsg(tf_global_map2robot_odom);
  global_map2robot_odom_.header.stamp = msg->header.stamp;
}

}  // namespace rm_chassis_controllers
