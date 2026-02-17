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
                .gun_offset_x = getParam(controller_nh, "gun_offset_x",0.2),
                .gun_offset_z = getParam(controller_nh, "gun_offset_z",0.0),
                .Cd_value = getParam(controller_nh, "Cd_value", 0.4),
                .Cd_distance = getParam(controller_nh, "Cd_distance",12.0),
                .Cd_slope = getParam(controller_nh, "Cd_slope", 0.0),
                .air_density = getParam(controller_nh, "air_density", 1.1),
                .g = getParam(controller_nh, "g", 9.81),
                .initial_vel = getParam(controller_nh, "initial_vel", 16.5),
                .max_simulation_time = getParam(controller_nh, "max_simulation_time", 2.5),
                .max_integration_step = getParam(controller_nh, "max_integration_step", 0.01),
                .newton_convergence_tol = getParam(controller_nh, "newton_convergence_tol", 2e-5),
                .finite_difference_eps = getParam(controller_nh, "finite_difference_eps", 1e-4),
                .max_newton_step = getParam(controller_nh, "max_newton_step", 0.04),
                .max_newton_iterations = getParam(controller_nh, "max_newton_iterations", 5)};
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
  launch2target.x = track_data.position.x - base2gimbal.transform.translation.x - config.gun_offset_x;
  launch2target.y = track_data.position.y - base2gimbal.transform.translation.y;
  launch2target.z = track_data.position.z - base2gimbal.transform.translation.z - config.gun_offset_z;
  double target_dis = std::sqrt(launch2target.x * launch2target.x + launch2target.y * launch2target.y);
  double target_hgt = launch2target.z;//0.737
  std::cout << "target_dis" << target_dis << std::endl << "target_hgt" << target_hgt << std::endl;
  yaw = std::atan2(launch2target.y, launch2target.x);
  double initial_pitch = -output_pitch_match_lut_.output(target_dis);
  std::cout << "initial_pitch: " << initial_pitch << std::endl;
  auto error_function = [this, &config, &target_dis, &target_hgt](double pitch_angle) -> double {
    return simulate(pitch_angle, config.initial_vel, target_dis, target_hgt);
  };
  double current_pitch = initial_pitch;
  double error = error_function(current_pitch);
  // Newton iteration
  for (int iter = 0; iter < config.max_newton_iterations; ++iter)
  {
    double h = config.finite_difference_eps;
    double f_plus  = error_function(current_pitch + h);
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
  BallisticConfig config = *config_rt_buffer_.readFromRT();
  std::array<double, 4> state = {{0.0, 0.0, initial_vel * std::cos(pitch_angle),initial_vel * std::sin(pitch_angle)}};
  double t = 0.0;
  double dt = config.max_integration_step;
  double x_prev = 0.0, z_prev = 0.0;
  double x_curr = 0.0, z_curr = 0.0;
  std::array<double, 4> k1{}, k2{}, k3{}, k4{}, temp{};
  auto systemEquation = [&](
          const std::array<double, 4>& state, std::array<double, 4>& stateDerivative){
    double vx = state[2], vz = state[3];
    double speed = std::sqrt(vx*vx + vz*vz);
    stateDerivative[0] = vx;
    stateDerivative[1] = vz;
    double fitting_Cd = config.Cd_value + config.Cd_slope * (target_dis - config.Cd_distance);
    double F_drag = 0.5 * config.air_density * fitting_Cd * M_PI * config.radius * config.radius * speed * speed;
    double drag_accel = F_drag / config.mass;
    stateDerivative[2] = -drag_accel * vx / speed;
    stateDerivative[3] = -config.g - drag_accel * vz / speed;
  };
  while (t < config.max_simulation_time && state[0] < target_dis)
  {
     x_prev = x_curr;
     z_prev = z_curr;
     x_curr = state[0];
     z_curr = state[1];
     // k1 = f(state)
     systemEquation(state, k1);
     // k2 = f(state + dt/2 * k1)
     for (int i = 0; i < 4; ++i)
       temp[i] = state[i] + 0.5 * dt * k1[i];
     systemEquation(temp, k2);
     // k3 = f(state + dt/2 * k2)
     for (int i = 0; i < 4; ++i)
       temp[i] = state[i] + 0.5 * dt * k2[i];
     systemEquation(temp, k3);
     // k4 = f(state + dt * k3)
     for (int i = 0; i < 4; ++i)
        temp[i] = state[i] + dt * k3[i];
     systemEquation(temp, k4);
     // state_new = state + dt/6 * (k1 + 2*k2 + 2*k3 + k4)
     for (int i = 0; i < 4; ++i)
        state[i] += dt / 6.0 * (k1[i] + 2.0*k2[i] + 2.0*k3[i] + k4[i]);
     t += dt;
  }
  double z_at_target;
  if (state[0] >= target_dis && std::abs(x_curr - x_prev) > 1e-6)
    z_at_target = z_prev + (z_curr - z_prev) * (target_dis - x_prev) / (x_curr - x_prev);
  else
    z_at_target = state[2];
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
    config.Cd_value = init_config.Cd_value;
    config.Cd_distance = init_config.Cd_distance;
    config.Cd_slope = init_config.Cd_slope;
    config.air_density = init_config.air_density;
    config.g = init_config.g;
    config.initial_vel = init_config.initial_vel;
    config.max_simulation_time = init_config.max_simulation_time;
    config.max_integration_step = init_config.max_integration_step;
    config.newton_convergence_tol = init_config.newton_convergence_tol;
    config.finite_difference_eps = init_config.finite_difference_eps;
    config.max_newton_step = init_config.max_newton_step;
    config.max_newton_iterations = init_config.max_newton_iterations;
    dynamic_reconfig_initialized_ = true;
  }
  BallisticConfig config_non_rt{
      .mass = config.mass,
      .radius = config.radius,
      .Cd_value = config.Cd_value,
      .Cd_distance = config.Cd_distance,
      .Cd_slope = config.Cd_slope,
      .air_density = config.air_density,
      .g = config.g,
      .initial_vel = config.initial_vel,
      .max_simulation_time = config.max_simulation_time,
      .max_integration_step = config.max_integration_step,
      .newton_convergence_tol = config.newton_convergence_tol,
      .finite_difference_eps = config.finite_difference_eps,
      .max_newton_step = config.max_newton_step,
      .max_newton_iterations = config.max_newton_iterations,
  };
  config_rt_buffer_.writeFromNonRT(config_non_rt);
}
}  // namespace rm_gimbal_controllers