//
// Created by ljyi on 25-9-20.
//

#include "rm_gimbal_controllers/ballistic_solver.h"

namespace rm_gimbal_controllers
{
BallisticSolver::BallisticSolver(ros::NodeHandle& controller_nh) : used_fallback_(false)
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
              .newton_convergence_tol = getParam(controller_nh, "newton_convergence_tol", 2e-5),
              .finite_difference_eps = getParam(controller_nh, "finite_difference_eps", 1e-6),
              .max_newton_step = getParam(controller_nh, "max_newton_step", 0.1),
              .max_newton_iterations = getParam(controller_nh, "max_newton_iterations", 2)};
  config_rt_buffer_.initRT(config_);
  XmlRpc::XmlRpcValue lut_config;
  if (controller_nh.getParam("output_pitch_match", lut_config))
    output_pitch_match_lut_.init(lut_config);
}

bool BallisticSolver::solver(const geometry_msgs::TransformStamped& odom2gimbal, const rm_msgs::TrackData& track_data, double& yaw, double& pitch)
{
  geometry_msgs::Vector3 launch2target;
  launch2target.x = 18;
  launch2target.y = 0;
  launch2target.z = 1.2 - 0.513468 +0.05;
  double target_dis = std::sqrt(launch2target.x * launch2target.x + launch2target.y * launch2target.y) - config_.gun_len;
  double target_hgt = launch2target.z;
  double initial_vel = target_dis <= 16.5 ? config_.initial_vel_near : config_.initial_vel_far;
  std::cout << "target_dis" << target_dis << std::endl << "target_hgt" << target_hgt << std::endl;
  std::cout << launch2target.x << "   " << launch2target.y << "   " << launch2target.z << std::endl;
  yaw = std::atan2(launch2target.y, launch2target.x);
  std::cout << "yaw: " << yaw << std::endl;
  // Use LUT to get initial guess.Originally defined: negative pitch indicates upward angle (head up)
  double initial_pitch = -output_pitch_match_lut_.output(target_dis);
  std::cout << "initial_pitch: " << initial_pitch << std::endl;
  // Error function for root finding
  auto error_function = [&](double pitch_angle) -> double {
    return simulate(pitch_angle, initial_vel, target_dis, target_hgt);
  };
  used_fallback_ = true;
  double current_pitch = initial_pitch;
  double error = error_function(current_pitch);
  // Check if initial guess is already good enough
  if (std::abs(error) < config_.newton_convergence_tol)
  {
    std::cout<<"initial pitch is well"<<std::endl;
    pitch = -initial_pitch;
    used_fallback_ = false;
    return true;
  }
  // Newton iteration
  for (int iter = 0; iter < config_.max_newton_iterations; ++iter)
  {
    double h = config_.finite_difference_eps;
    double f_plus = error_function(current_pitch + h);
    double jacobian = (f_plus - error) / h;
    double delta = error / jacobian;
    delta = (delta < -config_.max_newton_step) ? -config_.max_newton_step :
            (delta > config_.max_newton_step)  ? config_.max_newton_step :
                                                 delta;
    double update_pitch = current_pitch - delta;
    double update_error = error_function(update_pitch);
    // Check convergence after update
    if (std::abs(update_error) < config_.newton_convergence_tol)
    {
      pitch = -update_pitch;
      used_fallback_ = false;
      return true;
    }
    // Update for next iteration
    current_pitch = update_pitch;
    error = update_error;
    std::cout<<"newton"<<std::endl;
  }
  pitch = -initial_pitch;
  return true;
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
      //double Re = speed * 2.0 * config.radius / config.kinematic_viscosity;
      double Cd = 0.37;
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