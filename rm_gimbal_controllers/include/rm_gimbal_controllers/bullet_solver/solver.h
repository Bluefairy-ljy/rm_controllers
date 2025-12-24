//
// Created by guanlin on 25-4-3.
//

#pragma once

#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_buffer.h>
#include <dynamic_reconfigure/server.h>
#include <rm_common/ros_utilities.h>
#include <rm_common/linear_interpolation.h>
#include <rm_msgs/BulletSolverState.h>
#include <rm_msgs/ShootBeforehandCmd.h>
#include <rm_gimbal_controllers/BulletSolverConfig.h>
#include "rm_gimbal_controllers/bullet_solver/target_kinematics.h"
#include "rm_gimbal_controllers/bullet_solver/selector.h"

namespace rm_gimbal_controllers
{
struct Config
{
  double resistance_coff_qd_16, resistance_coff_qd_25, g, delay, dt, timeout;
  double min_switch_angle, max_track_target_vel, track_rotate_target_delay, track_move_target_delay;
  int min_switch_count;
};
class BulletSolver
{
public:
  explicit BulletSolver(ros::NodeHandle& controller_nh);
  void selectTarget(geometry_msgs::Point pos, geometry_msgs::Vector3 vel, double bullet_speed, double yaw, double v_yaw,
                    double r1, double r2, double dz, int armors_num);
  bool solve();
  double getYaw() const
  {
    return output_yaw_;
  }
  double getPitch() const
  {
    return -output_pitch_;
  }
  double getResistanceCoefficient(double bullet_speed) const;
  double getGimbalError(double yaw_real, double pitch_real);
  void getSelectedArmorPosAndVel(geometry_msgs::Point& armor_pos, geometry_msgs::Vector3& armor_vel);
  void publishState();
  void judgeShootBeforehand(const ros::Time& time, double v_yaw);
  void reconfigCB(rm_gimbal_controllers::BulletSolverConfig& config, uint32_t);
  ~BulletSolver() = default;

private:
  std::unique_ptr<TrackedTargetKinematic> tracked_target_kinematic_;
  std::unique_ptr<UntrackedTargetKinematic> untracked_target_kinematic_;
  std::unique_ptr<TargetSelector> target_selector_;
  std::unique_ptr<realtime_tools::RealtimePublisher<rm_msgs::BulletSolverState>> state_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<rm_msgs::ShootBeforehandCmd>> shoot_beforehand_cmd_pub_;
  geometry_msgs::Point target_pos_{};
  realtime_tools::RealtimeBuffer<Config> config_rt_buffer_;
  dynamic_reconfigure::Server<rm_gimbal_controllers::BulletSolverConfig>* d_srv_{};
  Config config_{};
  ros::Time switch_armor_time_{};
  rm_common::LinearInterp gimbal_switch_duration_;
  bool dynamic_reconfig_initialized_{};
  bool track_target_{};
  double output_yaw_{}, output_pitch_{};
  double bullet_speed_{}, resistance_coff_{}, fly_time_{};
  int target_armor_{};
  int current_switch_state_{};
};
}  // namespace rm_gimbal_controllers
