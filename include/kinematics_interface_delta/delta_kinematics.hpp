#pragma once

#include <array>
#include <optional>

namespace kinematics_interface_delta
{

struct DeltaGeometry
{
  double e; // end effector triangle side
  double f; // base triangle side
  double re; // forearm length
  double rf; // upper arm length
  double motor_z_offset;
};

class DeltaKinematics
{
public:
  explicit DeltaKinematics(const DeltaGeometry &g, bool debug = false);

  // Forward kinematics: given three joint angles (radians) returns (x,y,z) in meters
  // Returns std::nullopt if no valid solution
  std::optional<std::array<double,3>> forward(const std::array<double,3> &thetas) const;

  // Inverse kinematics: given (x,y,z) in meters returns joint angles (radians)
  // Returns std::nullopt if unreachable
  std::optional<std::array<double,3>> inverse(const std::array<double,3> &pos) const;

  // Calculate elbow positions given joint angles
  // Returns array of 3 elbow positions, each with (x, y, z) in meters
  std::array<std::array<double,3>, 3> calculate_elbow_positions(const std::array<double,3> &thetas) const;

private:
  DeltaGeometry g_;
  bool debug_ = false;

  // helper functions
  bool delta_calcAngleYZ(double x0, double y0, double z0, double &theta) const;
  std::optional<std::array<double,3>> delta_calcForward(double theta1, double theta2, double theta3) const;

  // allow turning on debug prints
  void set_debug(bool d) { debug_ = d; }
};

} // namespace
