//
// Created by ljyi on 25-9-20.
//

#pragma once

#include <boost/numeric/odeint/stepper/runge_kutta4.hpp>
#include <boost/numeric/odeint/integrate/integrate_const.hpp>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <realtime_tools/realtime_buffer.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <dynamic_reconfigure/server.h>
#include <rm_common/linear_interpolation.h>
#include <rm_common/ros_utilities.h>
#include <rm_common/ori_tool.h>
#include <rm_msgs/TrackData.h>

namespace rm_gimbal_controllers
{
struct BallisticConfig
{
  double mass, radius, gun_len, kinematic_viscosity, drag_coff, air_density, Re_crit, g;
  double initial_vel_near, initial_vel_far, max_simulation_time, max_integration_step;
  double newton_convergence_tol, finite_difference_eps, max_newton_step;
  int max_newton_iterations;
};

class BallisticSolver
{
public:
  explicit BallisticSolver(ros::NodeHandle& nh);
  ~BallisticSolver() = default;
  bool used_fallback_;
  /**
   * @brief Simulate trajectory and return vertical error at target distance
   * @param pitch_angle_ Launch pitch angle
   * @param initial_vel_ Initial velocity
   * @param target_dis Target distance
   * @param target_hgt Target height
   * @return Vertical error
   */
  double simulate(double pitch_angle_, double initial_vel_, double target_dis, double target_hgt);
  /**
   * @brief Solve for aiming yaw and pitch angles
   * @param track_data Target position in odom frame
   * @param yaw Output: desired yaw angle
   * @param pitch Output: desired pitch angle
   * @return true if solution converged
   */
  bool solver(const geometry_msgs::TransformStamped& odom2gimbal, const rm_msgs::TrackData& track_data, double& yaw, double& pitch);

private:
  BallisticConfig config_{};
  // real time buffer
  realtime_tools::RealtimeBuffer<BallisticConfig> config_rt_buffer_;
  // member variable
  rm_common::LinearInterp output_pitch_match_lut_;
  geometry_msgs::Point launch_point_;
  // ODE stepper
  typedef boost::numeric::odeint::runge_kutta4<std::vector<double>> stepper_;
};
}  // namespace rm_gimbal_controllers