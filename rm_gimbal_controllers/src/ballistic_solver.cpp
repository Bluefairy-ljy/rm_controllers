//
// Created by ljyi on 25-9-20.
//

#include "rm_gimbal_controllers/ballistic_solver.h"

namespace rm_gimbal_controllers
{
BallisticSolver::BallisticSolver(ros::NodeHandle& controller_nh)
{
  config_ = { .mass = getParam(controller_nh, "mass", 0.0445),
              .radius = getParam(controller_nh, "radius", 0.02125),
              .gun_len = getParam(controller_nh, "gun_len", 0.0),
              .kinematic_viscosity = getParam(controller_nh, "kinematic_viscosity", 1.5e-5),
              .drag_coff = getParam(controller_nh, "drag_coff", 0.5),
              .air_density = getParam(controller_nh, "air_density", 1.2),
              .Re_crit = getParam(controller_nh, "Re_crit", 1e5),
              .g = getParam(controller_nh, "g", 9.81),
              .initial_vel_near = getParam(controller_nh, "initial_vel_near", 14.0),
              .initial_vel_far = getParam(controller_nh, "initial_vel_far", 15.8),
              .max_simulation_time = getParam(controller_nh, "max_simulation_time", 5.0),
              .max_integration_step = getParam(controller_nh, "max_integration_step", 0.01),
              .abs_error_tolerance = getParam(controller_nh, "abs_error_tolerance", 1e-6),
              .rel_error_tolerance = getParam(controller_nh, "rel_error_tolerance", 1e-6) };
  config_rt_buffer_.initRT(config_);
  XmlRpc::XmlRpcValue lut_config;
  if (controller_nh.getParam("output_pitch_match", lut_config))
    output_pitch_match_lut_.init(lut_config);
}

bool BallisticSolver::solver(const geometry_msgs::TransformStamped& odom2gimbal, const rm_msgs::TrackData& track_data, double& yaw, double& pitch)
{
  double initial_pitch, final_pitch;
  geometry_msgs::Vector3 launch2target;
  launch2target.x = track_data.position.x - odom2gimbal.transform.translation.x;
  launch2target.y = track_data.position.y - odom2gimbal.transform.translation.y;
  launch2target.z = track_data.position.z - odom2gimbal.transform.translation.z;
  double target_dis = std::sqrt(launch2target.x * launch2target.x + launch2target.y * launch2target.y) - config_.gun_len;
  double target_hgt = launch2target.z;
  double initial_vel = target_dis <= 16.5 ? config_.initial_vel_near : config_.initial_vel_far;
  yaw = std::atan2(launch2target.y, launch2target.x);
  // Use LUT to get initial guess.Originally defined: negative pitch indicates upward angle (head up)
  initial_pitch = -output_pitch_match_lut_.output(target_dis);
  // Error function for root finding
  auto error_function = [&](double pitch_angle) -> double {
    return simulate(pitch_angle, initial_vel, target_dis, target_hgt);
  };
  bool success = false;
  try
  {
    // Set search interval for root finding
    double lower_bound = std::max(initial_pitch - 0.5, -M_PI/2 + 0.1);
    double upper_bound = std::min(initial_pitch + 0.5,  M_PI/2 - 0.1);
    boost::uintmax_t max_iterations = 3;
    // Perform solving: find pitch_angle ∈ [lower_bound, upper_bound] such that error_function(pitch_angle) ≈ 0
    auto result = boost::math::tools::toms748_solve(error_function, lower_bound, upper_bound,
                                                    boost::math::tools::eps_tolerance<double>(2), max_iterations);
    // Take the midpoint of the solution interval as the final solution because TOMS748 returns a convergence interval
    final_pitch = (result.first + result.second) / 2.0;
    double final_error = error_function(final_pitch);
    success = std::abs(final_error) < 0.2;
  }
  catch (const std::exception& e)
  {
    ROS_WARN_THROTTLE(1.0, "%s", e.what());
    final_pitch = initial_pitch;
    success = false;
  }
  pitch = -final_pitch;
  return success;
}

double BallisticSolver::simulate(double pitch_angle, double initial_vel, double target_dis, double target_hgt)
{
  if (initial_vel <= 0.0 || target_dis <= 0.0)
    return 1e6;
  BallisticConfig config = *config_rt_buffer_.readFromRT();
  std::vector<double> state = { 0.0, 0.0, 0.0, initial_vel * std::cos(pitch_angle), 0.0, initial_vel * std::sin(pitch_angle) };
  double t = 0.0;
  double t_max = config.max_simulation_time;
  double dt_max = config.max_integration_step;
  stepper_ stepper;
  double x_prev = 0.0, z_prev = 0.0;
  double x_curr = 0.0, z_curr = 0.0;
  bool cross_target = false;
  double z_at_target = 0.0;
  // Observer is called after each integration step to detect if target is overflown at the first time
  auto observer = [&](const std::vector<double>& s, double /* t */) {
    x_prev = x_curr;
    z_prev = z_curr;
    x_curr = s[0];
    z_curr = s[2];
    if (!cross_target && x_prev < target_dis && x_curr >= target_dis)
    {
      z_at_target = z_prev + (z_curr - z_prev) * (target_dis - x_prev) / (x_curr - x_prev);
      cross_target = true;
    }
  };
  auto system_func = [config](const std::vector<double>& state, std::vector<double>& dsdt, double t) {
    double vx = state[3], vy = state[4], vz = state[5];
    double speed = std::sqrt(vx * vx + vy * vy + vz * vz);
    dsdt.assign(6, 0.0);
    dsdt[0] = vx;
    dsdt[1] = vy;
    dsdt[2] = vz;
    double ax = 0.0, ay = 0.0, az = -config.g;
    if (speed > 1e-5)
    {
      double Re = speed * 2.0 * config.radius / config.kinematic_viscosity;
      double Cd = (Re < config.Re_crit) ? 0.44 * config.drag_coff : 0.2;
      double F_drag = 0.5 * config.air_density * Cd * M_PI * config.radius * config.radius * speed * speed;
      double inv_speed = 1.0 / speed;
      ax += (-F_drag * vx * inv_speed) / config.mass;
      ay += (-F_drag * vy * inv_speed) / config.mass;
      az += (-F_drag * vz * inv_speed) / config.mass;
    }
    dsdt[3] = ax;
    dsdt[4] = ay;
    dsdt[5] = az;
  };
 integrate_const(stepper, system_func, state, t, t_max, dt_max, observer);
  // After integration, if it's determined that the target was never overflown
  if (!cross_target)
  {
    // Use trajectory slope of last two points to linearly extrapolate to target distance
    double slope = (z_curr - z_prev) / (x_curr - x_prev);
    z_at_target = z_prev + slope * (target_dis - x_prev);
  }
  return z_at_target - target_hgt;
}
}  // namespace rm_gimbal_controllers