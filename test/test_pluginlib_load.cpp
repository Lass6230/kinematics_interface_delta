// Conditional pluginlib load test. Built only when pluginlib and kinematics_interface are available.
#ifdef KINEMATICS_INTERFACE_AVAILABLE

#include <gtest/gtest.h>
#include <pluginlib/class_loader.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <kinematics_interface/kinematics_interface/kinematics_interface.hpp>

TEST(PluginLoadTest, load_and_call_transform)
{
  rclcpp::init(0, nullptr);
  pluginlib::ClassLoader<kinematics_interface::KinematicsInterface> loader("kinematics_interface", "kinematics_interface::KinematicsInterface");
  ASSERT_NO_THROW({
    auto instance = loader.createUniqueInstance("kinematics_interface_delta/DeltaKinematicsPlugin");
    ASSERT_NE(instance, nullptr);
    // set default geometry via dynamic cast if available
    auto *delta = dynamic_cast<kinematics_interface_delta::DeltaKinematicsPlugin *>(instance.get());
    if (delta)
    {
      kinematics_interface_delta::DeltaGeometry g{0.045, 0.11812, 0.34, 0.17438, -0.025};
      delta->set_geometry(g);
      // initialize (node parameters pointer not provided)
      bool ok = instance->initialize("delta", nullptr, "ee");
      EXPECT_TRUE(ok);
      Eigen::VectorXd joint(3);
      joint << 0.1, 0.1, 0.1;
      Eigen::Isometry3d tf;
      bool tf_ok = instance->calculate_link_transform(joint, "ee", tf);
      EXPECT_TRUE(tf_ok);
    }
  });
  rclcpp::shutdown();
}

#endif
