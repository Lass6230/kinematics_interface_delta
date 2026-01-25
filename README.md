kinematics_interface_delta
=========================

Standalone C++ delta robot kinematics solver (analytic-ish) for ROS2 Jazzy projects.

This package contains:
- a small DeltaKinematics class (include + src)
- a test executable `test_delta_kinematics` that runs a forward->inverse roundtrip

Notes
-----
- Geometry units: the provided example uses mm converted to meters in the test. The DeltaGeometry struct stores lengths in meters.
- This is a standalone solver to be wrapped later into a `kinematics_interface` plugin.

Plugin configuration
--------------------
The plugin supports two ways to configure the delta geometry before initialization:

1) Environment variables (safe default; useful when loading via pluginlib)

  - KIN_DELTA_E (meters)
  - KIN_DELTA_F (meters)
  - KIN_DELTA_RE (meters)
  - KIN_DELTA_RF (meters)
  - KIN_DELTA_MOTOR_Z_OFFSET (meters)

  Example (bash):

  ```bash
  export KIN_DELTA_E=0.045
  export KIN_DELTA_F=0.11812
  export KIN_DELTA_RE=0.34
  export KIN_DELTA_RF=0.17438
  export KIN_DELTA_MOTOR_Z_OFFSET=-0.025
  ```

2) Programmatic: call `set_geometry()` on the plugin instance prior to use (e.g., from the node that manages plugin loading). This is the recommended approach when parameters are available in ROS and a node pointer is handy.

Note: If you want me to wire direct ROS parameter parsing in `initialize()` (reading parameters from the provided `NodeParametersInterface`), tell me which parameter names and I'll add it — I currently provide env-var fallback and `set_geometry()` for portability.

How to build (colcon workspace root):

  colcon build --packages-select kinematics_interface_delta

Then run the test binary (after sourcing install):

  ros2 run kinematics_interface_delta test_delta_kinematics
