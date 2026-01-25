#include <gtest/gtest.h>
#include "kinematics_interface_delta/delta_kinematics.hpp"

using namespace kinematics_interface_delta;

TEST(DeltaKinematicsTest, ForwardInverseRoundtrip)
{
  DeltaGeometry g;
  g.e = 45.0*1e-3;
  g.f = 118.12*1e-3;
  g.re = 340.0*1e-3;
  g.rf = 174.38*1e-3;
  g.motor_z_offset = -0.025;

  DeltaKinematics k(g, false);
  std::array<double,3> thetas = {0.2, 0.1, -0.15};
  auto pos_opt = k.forward(thetas);
  ASSERT_TRUE(pos_opt.has_value());
  auto pos = pos_opt.value();

  auto ik_opt = k.inverse(pos);
  ASSERT_TRUE(ik_opt.has_value());
  auto ik = ik_opt.value();

  for (int i=0;i<3;i++) {
    EXPECT_NEAR(thetas[i], ik[i], 1e-3);
  }
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
