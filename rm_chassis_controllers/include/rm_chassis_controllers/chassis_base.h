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

#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <rm_common/hardware_interface/robot_state_interface.h>
#include <realtime_tools/realtime_publisher.h>
#include <rm_common/filters/filters.h>
#include <effort_controllers/joint_velocity_controller.h>
#include <rm_msgs/ChassisCmd.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <string>

namespace rm_chassis_controllers
{
struct Command
{
  geometry_msgs::Twist cmd_vel_;
  rm_msgs::ChassisCmd cmd_chassis_;
  ros::Time stamp_;
};
template <typename... T>
class ChassisBase : public controller_interface::MultiInterfaceController<T...>
{
public:
  ChassisBase() = default;
  /** @brief Get and check params for covariances. Setup odometry realtime publisher + odom message constant fields.
   * init odom tf.
   *
   * @param robot_hw The robot hardware abstraction.
   * @param root_nh A NodeHandle in the root of the controller manager namespace. This is where the ROS interfaces are
   * setup (publishers, subscribers, services).
   * @param controller_nh A NodeHandle in the namespace of the controller. This is where the controller-specific
   * configuration resides.
   * @return True if initialization was successful and the controller
   * is ready to be started.
   */
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
  /** @brief Receive real_time command from manual. Execute different action according to current mode. Set
   * necessary params of chassis. Execute power limit.
   *
   * Receive real_time command from manual and check whether it is normally, if can not receive command from manual
   * for a while, chassis's velocity will be set zero to avoid out of control. Execute different action according
   * to current mode such as RAW, FOLLOW, TWIST.(Their specific usage will be explain in the next). UpdateOdom,Set
   * necessary params such as Acc and vel_tfed. Execute moving action and powerlimit.
   *
   * @param time The current time.
   * @param period The time passed since the last call to update.
   */
  void update(const ros::Time& time, const ros::Duration& period) override;

protected:
  /** @briefThe The mode RAW: original state.
   *
   *  The mode raw: original state. Linear velocity will be set zero to stop move.
   */
  void raw();
  /** @brief The mode FOLLOW: chassis will follow gimbal.
   *
   * The mode FOLLOW: The chassis's direct will follow gimbal's direct all the time.
   *
   * @param time The current time.
   * @param period The time passed since the last call to update.
   */
  void follow(const ros::Time& time, const ros::Duration& period);
  /** @brief The mode TWIST: Just moving chassis.
   *
   * The mode TWIST: Chassis will move independent and will not effect by gimbal's move.
   *
   * @param time The current time.
   * @param period The time passed since the last call to update.
   */
  void twist(const ros::Time& time, const ros::Duration& period);
  virtual void moveJoint(const ros::Time& time, const ros::Duration& period) = 0;
  virtual geometry_msgs::Twist odometry() = 0;
  /** @brief Init frame on base_link. Integral vel to pos and angle.
   *
   * @param time The current time.
   * @param period The time passed since the last call to update.
   */
  void updateOdom(const ros::Time& time, const ros::Duration& period);
  /** @brief Set chassis velocity to zero.
   */
  void recovery();
  /** @brief Transform tf velocity to base link frame.
   *
   * @param from The father frame.
   */
  void tfVelToBase(const std::string& from);
  /** @brief To limit the chassis power according to current power limit.
   *
   * Receive power limit from command. Set max_effort command to chassis to avoid exceed power limit.
   */
  void powerLimit();
  /** @brief Write current command from rm_msgs::ChassisCmd.
   *
   * @param msg This message contains various state parameter settings for basic chassis control
   */
  void cmdChassisCallback(const rm_msgs::ChassisCmdConstPtr& msg);
  /** @brief Write current command from  geometry_msgs::Twist.
   *
   * @param msg This expresses velocity in free space broken into its linear and angular parts.
   */
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);

  void initialize_parameters(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh,
                             ros::NodeHandle& controller_nh);
  void slamOdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  // void slamOdomCallback(const nav_msgs::Path::ConstPtr& msg);
  void localizationResultCallback(const geometry_msgs::TransformStamped::ConstPtr& msg);

protected:
  rm_control::RobotStateHandle robot_state_handle_{};
  hardware_interface::EffortJointInterface* effort_joint_interface_{};
  std::vector<hardware_interface::JointHandle> joint_handles_{};
  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry>> nav_odometry_rtpub_;
  realtime_tools::RealtimeBuffer<Command> cmd_rt_buffer_;
  realtime_tools::RealtimeBuffer<nav_msgs::Odometry> slam_rt_buffer_{};
  RampFilter<double>* ramp_x_{};
  RampFilter<double>* ramp_y_{};
  RampFilter<double>* ramp_w_{};
  ros::Subscriber cmd_vel_sub_;
  ros::Subscriber cmd_chassis_sub_;
  ros::Subscriber slam_odom_sub_;
  ros::Subscriber localization_result_sub_;
  rm_common::TfRtBroadcaster tf_broadcaster4global_map2robot_odom_{};
  rm_common::TfRtBroadcaster tf_broadcaster4robot_odom2imu_odom_{};
  geometry_msgs::TransformStamped global_map2robot_odom_{};
  geometry_msgs::TransformStamped global_map2lidar_odom_{};
  geometry_msgs::TransformStamped global_map2robot_base_{};
  geometry_msgs::TransformStamped robot_odom2robot_base_{};
  geometry_msgs::TransformStamped robot_odom2lidar_odom_{};
  geometry_msgs::TransformStamped robot_odom2imu_odom_{};
  geometry_msgs::TransformStamped lidar_odom2lidar_base_{};
  geometry_msgs::TransformStamped lidar_base2robot_base_{};
  geometry_msgs::TransformStamped robot_base2imu_odom_{};
  nav_msgs::Odometry wheel_odometry_{};

  tf2::Transform tf_robot_odom2lidar_odom_;
  tf2::Transform tf_lidar_odom2lidar_base_;
  tf2::Transform tf_lidar_base2robot_base_;
  tf2::Transform tf_robot_base2imu_odom_;
  tf2::Transform tf_robot_odom2imu_odom_;

  std::string slam_odom_topic_{ "/Odometry" };
  std::string localization_result_topic_{ "/hdl_global_localization/result" };
  std::string global_map_frame_id_{ "map" };
  std::string imu_odom_frame_id_{ "imu_odom" };
  std::string robot_odom_frame_id_{ "robot_odom" };
  std::string robot_base_frame_id_{ "base_link" };
  std::string lidar_odom_frame_id_{ "livox_odom" };
  std::string lidar_base_frame_id_{ "livox_frame" };

  std::string follow_source_frame_{};
  std::string command_source_frame_{};
  double wheel_radius_{ 0.02 };
  double publish_rate_{ 100 };
  double twist_angular_{ M_PI / 6 };
  double timeout_{ 0.1 };
  double effort_coeff_{ 6.0 };
  double velocity_coeff_{ 0.004 };
  double power_offset_{ 0.0 };
  double max_odom_vel_{ 0.0 };
  bool publish_map_tf_{ false };
  bool publish_odom_tf_{ false };
  bool enable_wheel_odom_{ false };
  bool enable_slam_odom_{ false };
  bool slam_odom_initialized_{ false };
  bool topic_update_{ false };
  bool state_changed_{ true };

  int state_ = RAW;

  ros::Time last_publish_time_;
  ros::Time last_debug_time_wheel_;
  ros::Time last_debug_time_slam_;
  ros::Time last_debug_time_imu_;

  geometry_msgs::Vector3 vel_cmd_{};  // x, y
  control_toolbox::Pid pid_follow_;

  Command cmd_struct_;

  enum
  {
    RAW,
    FOLLOW,
    TWIST
  };
};

}  // namespace rm_chassis_controllers
