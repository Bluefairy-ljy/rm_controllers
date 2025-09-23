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
              .max_simulation_time = getParam(controller_nh, "max_simulation_time", 5.0),
              .max_integration_step = getParam(controller_nh, "max_integration_step", 0.01),
              .abs_error_tolerance = getParam(controller_nh, "abs_error_tolerance", 1e-6),
              .rel_error_tolerance = getParam(controller_nh, "rel_error_tolerance", 1e-6) };
  config_rt_buffer_.initRT(config_);
  XmlRpc::XmlRpcValue lut_config;
  if (controller_nh.getParam("output_pitch_match", lut_config))
    output_pitch_match_lut_.init(lut_config);
  initial_vel_buffer_.initRT(0.0);
  initial_vel_sub_ = controller_nh.subscribe<std_msgs::Float32>("/shoot_data", 1, &BallisticSolver::initialVelCB, this);
}

bool BallisticSolver::solver(const rm_msgs::TrackData& track_data, double& yaw, double& pitch)
{
  double initial_vel = *initial_vel_buffer_.readFromRT();
  double initial_pitch, final_pitch;
  geometry_msgs::Vector3 launch2target;
  launch2target.x = track_data.position.x - launch_point_.x;
  launch2target.y = track_data.position.y - launch_point_.y;
  launch2target.z = track_data.position.z - launch_point_.z;
  double target_dis = std::sqrt(launch2target.x * launch2target.x + launch2target.y * launch2target.y);
  double target_hgt = launch2target.z;
  yaw = std::atan2(launch2target.y, launch2target.x);
  // Use LUT to get initial guess
  initial_pitch = output_pitch_match_lut_.output(target_dis);
  // Error function for root finding
  auto error_function = [&](double pitch_angle) -> double {
    return simulate(pitch_angle, initial_vel, target_dis, target_hgt);
  };
  bool success = false;
  try
  {
    double lower_bound = std::max(initial_pitch - 0.3, 0.1);
    double upper_bound = std::min(initial_pitch + 0.3, M_PI / 2 - 0.1);
    boost::uintmax_t max_iterations = 15;
    auto result = boost::math::tools::toms748_solve(error_function, lower_bound, upper_bound,
                                                    boost::math::tools::eps_tolerance<double>(5), max_iterations);
    final_pitch = (result.first + result.second) / 2.0;
    double final_error = error_function(final_pitch);
    success = std::abs(final_error) < 0.05;
  }
  catch (const std::exception& e)
  {
    ROS_WARN_THROTTLE(1.0, "%s", e.what());
    final_pitch = initial_pitch;
    success = false;
  }
  pitch = final_pitch;
  return success;
}

void BallisticSolver::getLaunchPoint(const geometry_msgs::TransformStamped& odom2gimbal, const geometry_msgs::TransformStamped& odom2base)
{
  BallisticConfig config = *config_rt_buffer_.readFromRT();
  double base_x = odom2base.transform.translation.x;
  double base_y = odom2base.transform.translation.y;
  double base_z = odom2base.transform.translation.z;
  tf2::Quaternion q_base;
  tf2::fromMsg(odom2base.transform.rotation, q_base);
  double roll_base, pitch_base, yaw_base;
  tf2::Matrix3x3(q_base).getRPY(roll_base, pitch_base, yaw_base);
  tf2::Quaternion q_gimbal;
  tf2::fromMsg(odom2gimbal.transform.rotation, q_gimbal);
  double roll_gimbal, pitch_gimbal, yaw_gimbal;
  tf2::Matrix3x3(q_gimbal).getRPY(roll_gimbal, pitch_gimbal, yaw_gimbal);
  double cos_pitch = std::cos(pitch_gimbal);
  double sin_pitch = std::sin(pitch_gimbal);
  launch_point_.x = base_x + config.gun_len * cos_pitch * std::cos(yaw_base);
  launch_point_.y = base_y + config.gun_len * cos_pitch * std::sin(yaw_base);
  launch_point_.z = base_z + config.gun_len * sin_pitch;
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
  auto controlled_stepper = boost::numeric::odeint::controlled_runge_kutta<stepper_>(
      boost::numeric::odeint::default_error_checker<double, boost::numeric::odeint::range_algebra,
                                                    boost::numeric::odeint::default_operations>(
          config.abs_error_tolerance, config.rel_error_tolerance));
  double x_prev = 0.0, z_prev = 0.0;
  double x_curr = 0.0, z_curr = 0.0;
  bool hit_target = false;
  double z_at_target = 0.0;
  auto observer = [&](const std::vector<double>& s, double /* t */) {
    x_prev = x_curr;
    z_prev = z_curr;
    x_curr = s[0];
    z_curr = s[2];
    if (!hit_target && x_prev < target_dis && x_curr >= target_dis)
    {
      z_at_target = z_prev + (z_curr - z_prev) * (target_dis - x_prev) / (x_curr - x_prev);
      hit_target = true;
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
  boost::numeric::odeint::integrate_const(controlled_stepper, system_func, state, t, t_max, dt_max, observer);
  if (!hit_target)
    z_at_target = state[2];
  return z_at_target - target_hgt;
}

void BallisticSolver::initialVelCB(const std_msgs::Float32::ConstPtr& msg)
{
  initial_vel_buffer_.writeFromNonRT(msg->data);
}
}  // namespace rm_gimbal_controllers