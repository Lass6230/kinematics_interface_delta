// Implementation of the DeltaKinematicsPlugin that maps the kinematics_interface API
// to the DeltaKinematics solver. Compiled only when KINEMATICS_INTERFACE_AVAILABLE is set.

#ifdef KINEMATICS_INTERFACE_AVAILABLE

#include "kinematics_interface_delta/delta_plugin.hpp"
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <memory>

namespace kinematics_interface_delta
{

bool DeltaKinematicsPlugin::initialize(const std::string & /*name*/, std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> node_parameters, const std::string & /*root_link*/)
{
  // For now use geometry_ defaults or allow caller to call set_geometry().
  // First, prefer ROS parameters from the provided NodeParametersInterface when available.
  bool updated = false;
  if (node_parameters)
  {
    try
    {
      rclcpp::Parameter p;
      // Try simple names first, then fully qualified under package name.
      const std::vector<std::pair<std::string, double *>> param_map = {
        {"e", &geometry_.e},
        {"f", &geometry_.f},
        {"re", &geometry_.re},
        {"rf", &geometry_.rf},
        {"motor_z_offset", &geometry_.motor_z_offset},
      };

      for (const auto &kv : param_map)
      {
        const std::string &name = kv.first;
        double *dest = kv.second;
        // try direct
        if (node_parameters->get_parameter(name, p))
        {
          try { *dest = p.as_double(); updated = true; }
          catch (...) { /* ignore */ }
          continue;
        }
        // try namespaced under package
        std::string pkg_name = std::string("kinematics_interface_delta.") + name;
        if (node_parameters->get_parameter(pkg_name, p))
        {
          try { *dest = p.as_double(); updated = true; }
          catch (...) { /* ignore */ }
          continue;
        }
      }
      if (updated)
      {
        RCLCPP_INFO(rclcpp::get_logger("DeltaKinematicsPlugin"), "Delta geometry set from node parameters: e=%f f=%f re=%f rf=%f motor_z_offset=%f",
                    geometry_.e, geometry_.f, geometry_.re, geometry_.rf, geometry_.motor_z_offset);
      }
    }
    catch (const std::exception &ex)
    {
      RCLCPP_WARN(rclcpp::get_logger("DeltaKinematicsPlugin"), "Error reading node parameters: %s", ex.what());
    }
  }

  // Next, fallback to environment variables if nothing was provided via parameters.
  if (!updated)
  {
    try
    {
      const char *env_e = std::getenv("KIN_DELTA_E");
      const char *env_f = std::getenv("KIN_DELTA_F");
      const char *env_re = std::getenv("KIN_DELTA_RE");
      const char *env_rf = std::getenv("KIN_DELTA_RF");
      const char *env_mz = std::getenv("KIN_DELTA_MOTOR_Z_OFFSET");
      if (env_e) { geometry_.e = std::stod(env_e); updated = true; }
      if (env_f) { geometry_.f = std::stod(env_f); updated = true; }
      if (env_re) { geometry_.re = std::stod(env_re); updated = true; }
      if (env_rf) { geometry_.rf = std::stod(env_rf); updated = true; }
      if (env_mz) { geometry_.motor_z_offset = std::stod(env_mz); updated = true; }
      if (updated)
      {
        RCLCPP_INFO(rclcpp::get_logger("DeltaKinematicsPlugin"), "Delta geometry set from environment: e=%f f=%f re=%f rf=%f motor_z_offset=%f",
                    geometry_.e, geometry_.f, geometry_.re, geometry_.rf, geometry_.motor_z_offset);
      }
    }
    catch (const std::exception &ex)
    {
      RCLCPP_WARN(rclcpp::get_logger("DeltaKinematicsPlugin"), "Failed to parse KIN_DELTA_* env vars: %s", ex.what());
    }
  }

  solver_.reset(new DeltaKinematics(geometry_, false));  // Disable debug spam
  initialized_ = true;
  return true;
}

void DeltaKinematicsPlugin::set_geometry(const kinematics_interface_delta::DeltaGeometry &g)
{
  geometry_ = g;
  if (solver_)
  {
    solver_.reset(new DeltaKinematics(geometry_, false));  // Disable debug spam
  }
}

std::array<std::array<double,3>, 3> DeltaKinematicsPlugin::calculate_elbow_positions(const Eigen::VectorXd &joint_angles)
{
  if (!initialized_ || !solver_ || joint_angles.size() < 3)
  {
    // Return zeros if not properly initialized
    return {{{0,0,0}, {0,0,0}, {0,0,0}}};
  }
  
  std::array<double,3> thetas = {joint_angles[0], joint_angles[1], joint_angles[2]};
  return solver_->calculate_elbow_positions(thetas);
}

bool DeltaKinematicsPlugin::calculate_link_transform(const Eigen::VectorXd &joint_angles, const std::string &link_name, Eigen::Isometry3d &transform)
{
  if (!initialized_ || !solver_)
  {
    RCLCPP_ERROR(rclcpp::get_logger("DeltaKinematicsPlugin"), "calculate_link_transform: NOT INITIALIZED or no solver! init=%d solver=%p", initialized_, (void*)solver_.get());
    return false;
  }
  if (joint_angles.size() < 3)
  {
    RCLCPP_ERROR(rclcpp::get_logger("DeltaKinematicsPlugin"), "calculate_link_transform: joint_angles size %zu < 3", joint_angles.size());
    return false;
  }

  // Check if requesting base/world frame transform (identity)
  if (link_name == "base" || link_name == "world" || link_name == "base_link")
  {
    transform = Eigen::Isometry3d::Identity();
    return true;
  }

  // Check if requesting end effector transform
  if (link_name != "ee" && link_name != "tool" && link_name != "end_effector")
  {
    // unsupported link name for this simple plugin
    RCLCPP_ERROR(rclcpp::get_logger("DeltaKinematicsPlugin"), "calculate_link_transform: unsupported link_name '%s'", link_name.c_str());
    return false;
  }

  // Calculate forward kinematics for end effector
  std::array<double, 3> thetas{joint_angles[0], joint_angles[1], joint_angles[2]};
  auto p = solver_->forward(thetas);
  if (!p)
  {
    RCLCPP_ERROR(rclcpp::get_logger("DeltaKinematicsPlugin"), "calculate_link_transform: FK returned nullopt for thetas=[%.3f,%.3f,%.3f]", thetas[0], thetas[1], thetas[2]);
    return false;
  }

  RCLCPP_INFO(rclcpp::get_logger("DeltaKinematicsPlugin"), "FK SUCCESS in plugin: [%.6f, %.6f, %.6f]", (*p)[0], (*p)[1], (*p)[2]);

  transform = Eigen::Isometry3d::Identity();
  transform.translate(Eigen::Vector3d((*p)[0], (*p)[1], (*p)[2]));
  return true;
}

bool DeltaKinematicsPlugin::calculate_jacobian(const Eigen::VectorXd &joint_angles, const std::string & /*reference_frame*/, Eigen::Matrix<double, 6, -1> &jacobian_out)
{
  if (!initialized_ || !solver_)
    return false;
  if (joint_angles.size() < 3)
    return false;

  const double eps = 1e-7;
  std::array<double, 3> base_thetas{joint_angles[0], joint_angles[1], joint_angles[2]};
  auto p0 = solver_->forward(base_thetas);
  if (!p0)
    return false;

  Eigen::Matrix<double, 3, 3> Jlin; // d(x,y,z)/d(theta)
  for (int i = 0; i < 3; ++i)
  {
    std::array<double, 3> pert = base_thetas;
    pert[i] += eps;
    auto pi = solver_->forward(pert);
    if (!pi)
      return false;
    Jlin(0, i) = ((*pi)[0] - (*p0)[0]) / eps;
    Jlin(1, i) = ((*pi)[1] - (*p0)[1]) / eps;
    Jlin(2, i) = ((*pi)[2] - (*p0)[2]) / eps;
  }

  // Build 6x3 jacobian: top 3 rows linear, bottom 3 rows angular (zero for delta)
  jacobian_out.resize(6, 3);
  jacobian_out.setZero();
  jacobian_out.block<3, 3>(0, 0) = Jlin;
  return true;
}

bool DeltaKinematicsPlugin::calculate_jacobian_inverse(const Eigen::VectorXd &joint_angles, const std::string &reference_frame, Eigen::Matrix<double, -1, 6> &jacobian_inverse_out)
{
  (void)reference_frame;
  if (!initialized_ || !solver_)
    return false;
  if (joint_angles.size() < 3)
    return false;

  Eigen::Matrix<double, 6, -1> J;
  if (!calculate_jacobian(joint_angles, reference_frame, J))
    return false;

  // Extract linear 3x3 block
  Eigen::Matrix3d A = J.block<3, 3>(0, 0);

  // Compute damped pseudoinverse of A via SVD with Tikhonov regularization.
  // This improves stability near singularities. Damping factor can be overridden
  // with the environment variable KIN_DELTA_JAC_PSEUDO_DAMP (meters^2).
  double damp = 1e-6;
  const char *env_d = std::getenv("KIN_DELTA_JAC_PSEUDO_DAMP");
  if (env_d)
  {
    try { damp = std::stod(env_d); } catch (...) { /* ignore invalid */ }
  }
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Vector3d singular = svd.singularValues();
  Eigen::Matrix3d S_pinv = Eigen::Matrix3d::Zero();
  for (int i = 0; i < 3; ++i)
  {
    double s = singular(i);
    // damped inverse of singular value: s / (s^2 + damp)
    if (s > 0.0)
      S_pinv(i, i) = s / (s * s + damp);
  }
  Eigen::Matrix3d A_pinv = svd.matrixV() * S_pinv * svd.matrixU().transpose();

  // Build full inverse mapping from 6-vector (spatial) to 3 joint deltas: [A_pinv | 0]
  jacobian_inverse_out.resize(3, 6);
  jacobian_inverse_out.setZero();
  jacobian_inverse_out.block<3, 3>(0, 0) = A_pinv;
  // angular part remains zero
  return true;
}

bool DeltaKinematicsPlugin::convert_cartesian_deltas_to_joint_deltas(const Eigen::VectorXd &joint_angles,
                                                                     const Eigen::Matrix<double, 6, 1> &cart_delta,
                                                                     const std::string &reference_frame,
                                                                     Eigen::VectorXd &joint_delta)
{
  if (!initialized_ || !solver_)
    return false;
  if (joint_angles.size() < 3)
    return false;

  Eigen::Matrix<double, -1, 6> Jinv;
  if (!calculate_jacobian_inverse(joint_angles, reference_frame, Jinv))
    return false;

  Eigen::Vector3d jd = Jinv * cart_delta;
  joint_delta = Eigen::VectorXd(3);
  joint_delta << jd(0), jd(1), jd(2);
  return true;
}

bool DeltaKinematicsPlugin::convert_joint_deltas_to_cartesian_deltas(const Eigen::VectorXd &joint_angles,
                                                                     const Eigen::VectorXd &joint_delta_in,
                                                                     const std::string &reference_frame,
                                                                     Eigen::Matrix<double, 6, 1> &cart_delta_out)
{
  (void)reference_frame;
  if (!initialized_ || !solver_)
    return false;
  if (joint_angles.size() < 3 || joint_delta_in.size() < 3)
    return false;

  Eigen::Matrix<double, 6, -1> J;
  if (!calculate_jacobian(joint_angles, reference_frame, J))
    return false;

  Eigen::Vector3d jd;
  jd << joint_delta_in(0), joint_delta_in(1), joint_delta_in(2);

  Eigen::Matrix<double, 6, 1> out = J * jd;
  cart_delta_out = out;
  return true;
}

} // namespace kinematics_interface_delta

// Register the plugin with pluginlib
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  kinematics_interface_delta::DeltaKinematicsPlugin,
  kinematics_interface::KinematicsInterface)

#endif // KINEMATICS_INTERFACE_AVAILABLE
