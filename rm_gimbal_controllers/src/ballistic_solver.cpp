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
                .gun_len = getParam(controller_nh, "gun_len", 0.08),
                .drag_coff= getParam(controller_nh, "drag_coff", 1.2),
                .Cd = getParam(controller_nh, "Cd", 0.4),
                .air_density = getParam(controller_nh, "air_density", 1.2),
                .g = getParam(controller_nh, "g", 9.81),
                .initial_vel_near = getParam(controller_nh, "initial_vel_near", 14.0),
                .initial_vel_far = getParam(controller_nh, "initial_vel_far", 15.7),
                .max_simulation_time = getParam(controller_nh, "max_simulation_time", 2.5),
                .max_integration_step = getParam(controller_nh, "max_integration_step", 0.01),
                .newton_convergence_tol = getParam(controller_nh, "newton_convergence_tol", 2e-5),
                .finite_difference_eps = getParam(controller_nh, "finite_difference_eps", 1e-4),
                .max_newton_step = getParam(controller_nh, "max_newton_step", 0.04),
                .debug_x = getParam(controller_nh, "debug_x", 0.0),
                .debug_y = getParam(controller_nh, "debug_y", 0.0),
                .debug_z = getParam(controller_nh, "debug_z", 0.0),
                .max_newton_iterations = getParam(controller_nh, "max_newton_iterations", 2)};
  used_debug_=getParam(controller_nh, "used_debug", false);
  config_rt_buffer_.initRT(config_);
  XmlRpc::XmlRpcValue lut_config;
  if (controller_nh.getParam("output_pitch_match", lut_config))
    output_pitch_match_lut_.init(lut_config);
  d_srv_ = new dynamic_reconfigure::Server<rm_gimbal_controllers::BallisticSolverConfig>(controller_nh);
  dynamic_reconfigure::Server<rm_gimbal_controllers::BallisticSolverConfig>::CallbackType cb =
      [this](auto&& PH1, auto&& PH2) { reconfigCB(PH1, PH2); };
  d_srv_->setCallback(cb);
}

bool BallisticSolver::solver(const geometry_msgs::TransformStamped& base2gimbal, const rm_msgs::TrackData& track_data, double& yaw, double& pitch)
{
  BallisticConfig config = *config_rt_buffer_.readFromRT();
  geometry_msgs::Vector3 launch2target;
  if (used_debug_) {
    launch2target.x = config.debug_x;
    launch2target.y = config.debug_y;
    launch2target.z = config.debug_z;
  }else {
    launch2target.x = track_data.position.x - base2gimbal.transform.translation.x-0.20;
    launch2target.y = track_data.position.y - base2gimbal.transform.translation.y;
    launch2target.z = track_data.position.z - base2gimbal.transform.translation.z;
  }
  double target_dis = std::sqrt(launch2target.x * launch2target.x + launch2target.y * launch2target.y);
  double target_hgt = launch2target.z-0.10;
  double initial_vel = target_dis <= 16.5 ? config.initial_vel_near : config.initial_vel_far;
  std::cout << "target_dis" << target_dis << std::endl << "target_hgt" << target_hgt << std::endl;
  std::cout << launch2target.x << "   " << launch2target.y << "   " << launch2target.z << std::endl;
  yaw = std::atan2(launch2target.y, launch2target.x);
  // Use LUT to get initial guess.Originally defined: negative pitch indicates upward angle (head up)
  double initial_pitch = -output_pitch_match_lut_.output(target_dis);
  std::cout << "initial_pitch: " << initial_pitch << std::endl;
  // Error function for root finding
  auto error_function = [&](double pitch_angle) -> double {
    return simulate(pitch_angle, initial_vel, target_dis, target_hgt);
  };
  double current_pitch = initial_pitch;
  double error = error_function(current_pitch);
  // Check if initial guess is already good enough
  if (std::abs(error) < config.newton_convergence_tol)
  {
    std::cout<<"initial pitch is well"<<std::endl;
    pitch = -initial_pitch;
    used_fallback_ = false;
    return true;
  }
  // Newton iteration
  for (int iter = 0; iter < config.max_newton_iterations; ++iter)
  {
    double h = config.finite_difference_eps;
    double f_plus  = error_function(current_pitch + h);
    //double f_minus = error_function(current_pitch - h);
    //double jacobian = (f_plus - f_minus) / (2 * h);
    double jacobian= (f_plus - error) / h;
    double delta = error / jacobian;
    delta = (delta < -config.max_newton_step) ? -config.max_newton_step :
            (delta > config.max_newton_step)  ? config.max_newton_step :
                                                 delta;
    double update_pitch = current_pitch - delta;
    double update_error = error_function(update_pitch);
    // Check convergence after update
    if (std::abs(update_error) < config.newton_convergence_tol)
    {
      std::cout<<"update pitch is well"<<std::endl;
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
  std::array<double, 6> state = {{0.0, 0.0, 0.0,initial_vel * std::cos(pitch_angle),0.0,initial_vel * std::sin(pitch_angle)}};
  double t = 0.0;
  double t_max = config.max_simulation_time;
  double dt = config.max_integration_step;
  stepper_ stepper;
  double x_prev = 0.0, z_prev = 0.0;
  double x_curr = 0.0, z_curr = 0.0;
  double z_at_target = 0.0;
  auto system_func = [config](const std::array<double, 6>& state, std::array<double, 6>& dsdt, double t) {
    double vx = state[3], vy = state[4], vz = state[5];
    double speed = std::sqrt(vx * vx + vy * vy + vz * vz);
    dsdt[0] = vx;
    dsdt[1] = vy;
    dsdt[2] = vz;
    double ax = 0.0, ay = 0.0, az = -config.g;
    if (speed > 1e-5) {
      double F_drag = config.drag_coff * 0.5 * config.air_density * config.Cd * M_PI * config.radius * config.radius * speed * speed;
      double inv_speed = 1.0 / speed;
      ax += (-F_drag * vx * inv_speed) / config.mass;
      ay += (-F_drag * vy * inv_speed) / config.mass;
      az += (-F_drag * vz * inv_speed) / config.mass;
    }
    dsdt[3] = ax;
    dsdt[4] = ay;
    dsdt[5] = az;
  };
  while (t < t_max && state[0] < target_dis)
  {
    x_prev = x_curr;
    z_prev = z_curr;
    x_curr = state[0];
    z_curr = state[2];
    stepper.do_step(system_func, state, t, dt);
    t += dt;
  }
  if (state[0] >= target_dis) {
    z_at_target = z_prev + (z_curr - z_prev) * (target_dis - x_prev) / (x_curr - x_prev);
    std::cout<<t<<std::endl;
  }
  double a=z_at_target-target_hgt;
  std::cout << "error = " << a << std::endl;
  return z_at_target - target_hgt;
}

void BallisticSolver::reconfigCB(rm_gimbal_controllers::BallisticSolverConfig& config, uint32_t /*unused*/)
{
  ROS_INFO("[Ballistic Solver] Dynamic params changed");
  if (!dynamic_reconfig_initialized_)
  {
    BallisticConfig init_config = *config_rt_buffer_.readFromNonRT();  // config init from YAML
    config.mass = init_config.mass;
    config.radius = init_config.radius;
    config.gun_len = init_config.gun_len;
    config.drag_coff = init_config.drag_coff;
    config.Cd = init_config.Cd;
    config.air_density = init_config.air_density;
    config.g = init_config.g;
    config.initial_vel_near = init_config.initial_vel_near;
    config.initial_vel_far = init_config.initial_vel_far;
    config.max_simulation_time = init_config.max_simulation_time;
    config.max_integration_step = init_config.max_integration_step;
    config.newton_convergence_tol = init_config.newton_convergence_tol;
    config.finite_difference_eps = init_config.finite_difference_eps;
    config.max_newton_step = init_config.max_newton_step;
    config.debug_x = init_config.debug_x;
    config.debug_y = init_config.debug_y;
    config.debug_z = init_config.debug_z;
    config.max_newton_iterations = init_config.max_newton_iterations;
    dynamic_reconfig_initialized_ = true;
  }
  BallisticConfig config_non_rt{
      .mass = config.mass,
      .radius = config.radius,
      .gun_len = config.gun_len,
      .drag_coff = config.drag_coff,
      .Cd = config.Cd,
      .air_density = config.air_density,
      .g = config.g,
      .initial_vel_near = config.initial_vel_near,
      .initial_vel_far = config.initial_vel_far,
      .max_simulation_time = config.max_simulation_time,
      .max_integration_step = config.max_integration_step,
      .newton_convergence_tol = config.newton_convergence_tol,
      .finite_difference_eps = config.finite_difference_eps,
      .max_newton_step = config.max_newton_step,
      .debug_x = config.debug_x,
      .debug_y = config.debug_y,
      .debug_z = config.debug_z,
      .max_newton_iterations = config.max_newton_iterations,
  };
  config_rt_buffer_.writeFromNonRT(config_non_rt);
}
}  // namespace rm_gimbal_controllers