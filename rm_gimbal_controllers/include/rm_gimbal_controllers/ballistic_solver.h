//
// Created by ljyi on 25-9-20.
//

#pragma once

#include <boost/numeric/odeint/stepper/runge_kutta_dopri5.hpp>
#include <boost/numeric/odeint/integrate/integrate_const.hpp>
#include <boost/numeric/odeint/stepper/generation/generation_controlled_runge_kutta.hpp>
#include <boost/math/tools/toms748_solve.hpp>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_buffer.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <dynamic_reconfigure/server.h>
#include <rm_common/hardware_interface/robot_state_interface.h>
#include <rm_common/linear_interpolation.h>
#include <rm_common/ros_utilities.h>
#include <rm_msgs/TrackData.h>

namespace rm_gimbal_controllers
{
struct BallisticConfig
{
  double mass, radius, gun_len, kinematic_viscosity, drag_coff, air_density, Re_crit, g;
  double max_simulation_time, max_integration_step, abs_error_tolerance, rel_error_tolerance;
};

class BallisticSolver
{
public:
  explicit BallisticSolver(ros::NodeHandle& nh);
  ~BallisticSolver() = default;
  /**
   * @brief Calculate launch point
   * @param odom2gimbal Transform from odometry frame to gimbal frame, including position and orientation.
   * @param odom2base Transform from odometry frame to base_link frame, including position and orientation.
   */
  void getLaunchPoint(const geometry_msgs::TransformStamped& odom2gimbal,const geometry_msgs::TransformStamped& odom2base);
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
  bool solver(const rm_msgs::TrackData& track_data, double& yaw, double& pitch);
  // Callback function
  void initialVelCB(const std_msgs::Float32::ConstPtr& msg);

private:
  BallisticConfig config_;
  // ros subscriber
  ros::Subscriber initial_vel_sub_;
  // real time buffer
  realtime_tools::RealtimeBuffer<BallisticConfig> config_rt_buffer_;
  realtime_tools::RealtimeBuffer<double> initial_vel_buffer_;
  // member variable
  rm_common::LinearInterp output_pitch_match_lut_;
  geometry_msgs::Point launch_point_;
  // ODE stepper
  typedef boost::numeric::odeint::runge_kutta_dopri5<std::vector<double>> stepper_;
};
}  // namespace rm_gimbal_controllers