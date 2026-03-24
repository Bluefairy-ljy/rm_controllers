//
// Created by ljyi on 25-9-20.
//

#pragma once

#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <realtime_tools/realtime_buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <dynamic_reconfigure/server.h>
#include <rm_gimbal_controllers/BallisticSolverConfig.h>
#include <rm_common/linear_interpolation.h>
#include <rm_common/ros_utilities.h>
#include <rm_common/ori_tool.h>
#include <rm_msgs/ShootData.h>
#include <rm_msgs/TrackData.h>

namespace rm_gimbal_controllers
{
struct BallisticConfig
{
  double mass, radius, gun_offset_x, gun_offset_z, Cd_value, Cd_distance, Cd_slope, air_density, g;
  double initial_vel, rk4_simulate_step;
  double newton_convergence_tol, newton_pitch_epsilon, max_newton_step;
  int max_newton_iterations;
};

class BallisticSolver
{
public:
  explicit BallisticSolver(ros::NodeHandle& nh);
  ~BallisticSolver() = default;
  bool used_fallback_ = true;
  double simulate(double pitch_angle_, double initial_vel_, double target_dis, double target_hgt);
  bool solver(const geometry_msgs::TransformStamped& base2gimbal, const geometry_msgs::Point& base2target, const std_msgs::Float64& terrain_data, double& yaw, double& pitch);
  void reconfigCB(rm_gimbal_controllers::BallisticSolverConfig& config, uint32_t);

private:
  dynamic_reconfigure::Server<rm_gimbal_controllers::BallisticSolverConfig>* d_srv_{};
  BallisticConfig config_{};
  realtime_tools::RealtimeBuffer<BallisticConfig> config_rt_buffer_;
  rm_common::LinearInterp output_pitch_match_lut_;
  bool dynamic_reconfig_initialized_{};
};
}  // namespace rm_gimbal_controllers