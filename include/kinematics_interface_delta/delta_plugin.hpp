#pragma once

#include "kinematics_interface_delta/delta_kinematics.hpp"

// This header declares a plugin wrapper class for kinematics_interface.
// It is compiled only when the upstream kinematics_interface package is available.

#ifdef KINEMATICS_INTERFACE_AVAILABLE
#include <kinematics_interface/kinematics_interface/kinematics_interface.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace kinematics_interface_delta
{

class DeltaKinematicsPlugin : public kinematics_interface::KinematicsInterface
{
public:
  DeltaKinematicsPlugin() = default;
  ~DeltaKinematicsPlugin() override = default;
  // Map the kinematics_interface API to the DeltaKinematics implementation.
  bool initialize(const std::string &name,
                  std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> node_parameters,
                  const std::string &root_link) override;

  bool convert_cartesian_deltas_to_joint_deltas(const Eigen::VectorXd &joint_angles,
                                                const Eigen::Matrix<double, 6, 1> &cart_delta,
                                                const std::string &reference_frame,
                                                Eigen::VectorXd &joint_delta) override;

  bool convert_joint_deltas_to_cartesian_deltas(const Eigen::VectorXd &joint_angles,
                                                const Eigen::VectorXd &joint_delta_in,
                                                const std::string &reference_frame,
                                                Eigen::Matrix<double, 6, 1> &cart_delta_out) override;

  bool calculate_link_transform(const Eigen::VectorXd &joint_angles,
                                const std::string &link_name,
                                Eigen::Isometry3d &transform) override;

  bool calculate_jacobian(const Eigen::VectorXd &joint_angles,
                          const std::string &reference_frame,
                          Eigen::Matrix<double, 6, -1> &jacobian_out) override;

  bool calculate_jacobian_inverse(const Eigen::VectorXd &joint_angles,
                                  const std::string &reference_frame,
                                  Eigen::Matrix<double, -1, 6> &jacobian_inverse_out) override;

  // Allow users to set geometry explicitly (useful when parameters are supplied elsewhere)
  void set_geometry(const kinematics_interface_delta::DeltaGeometry &g);

  // Calculate elbow positions for visualization/control
  std::array<std::array<double,3>, 3> calculate_elbow_positions(const Eigen::VectorXd &joint_angles);

private:
  // delta solver instance (constructed during initialize or via set_geometry)
  std::unique_ptr<kinematics_interface_delta::DeltaKinematics> solver_;
  kinematics_interface_delta::DeltaGeometry geometry_{0.045, 0.11812, 0.34, 0.17438, -0.025}; // defaults in meters
  bool initialized_ = false;
};

} // namespace
#endif
