#include "kinematics_interface_delta/delta_kinematics.hpp"
#include <iostream>
#include <cmath>
#include <vector>

int main(int argc, char **argv)
{
  using namespace kinematics_interface_delta;

  DeltaGeometry g;
  g.e = 45.0; // mm in original — we'll assume mm and convert to meters in code if needed
  g.f = 118.12;
  g.re = 340.0;
  g.rf = 174.38;
  // user provided motor_z_offset was -0.025 (meters)
  g.motor_z_offset = -0.025; // meters

  // Convert mm to meters for lengths (common unit)
  g.e *= 1e-3;
  g.f *= 1e-3;
  g.re *= 1e-3;
  g.rf *= 1e-3;

  // Enable debug prints to help diagnose failures
  DeltaKinematics k(g, true);

  std::vector<std::array<double,3>> test_thetas = {
    {0.1, 0.1, 0.1},
    {0.2, 0.3, -0.15},
    {-0.2, 0.1, 0.1},
    {0.5, -0.4, 0.2}
  };

  std::cout << "Running forward->inverse sweep for several theta sets" << std::endl;
  for (size_t i=0;i<test_thetas.size();++i) {
    auto thetas = test_thetas[i];
    auto pos_opt = k.forward(thetas);
    if (!pos_opt) {
      std::cout << "Case " << i << ": Forward kinematics failed for thetas: " << thetas[0] << ", " << thetas[1] << ", " << thetas[2] << "\n";
      continue;
    }
    auto pos = *pos_opt;
    std::cout << "Case " << i << " Forward: x=" << pos[0] << " y=" << pos[1] << " z=" << pos[2] << "\n";

    auto ik_opt = k.inverse(pos);
    if (!ik_opt) {
      std::cout << "Case " << i << " Inverse kinematics failed for pos above.\n";
      continue;
    }
    auto ik = *ik_opt;
    std::cout << "Case " << i << " Inverse: t1=" << ik[0] << " t2=" << ik[1] << " t3=" << ik[2] << "\n";
    for (int j=0;j<3;++j) {
      std::cout << "  theta" << j+1 << " diff = " << std::fabs(thetas[j] - ik[j]) << " rad\n";
    }
  }

  std::cout << "Now testing inverse on a z-sweep at x=y=0" << std::endl;
  for (double z = -0.01; z >= -0.06; z -= 0.005) {
    std::array<double,3> p = {0.0, 0.0, z};
    auto ik_opt = k.inverse(p);
    if (!ik_opt) {
      std::cout << "z=" << z << " -> inverse failed\n";
    } else {
      auto ik = *ik_opt;
      std::cout << "z=" << z << " -> t = (" << ik[0] << ", " << ik[1] << ", " << ik[2] << ")\n";
    }
  }

  std::cout << "Sweep done" << std::endl;
  return 0;
}
