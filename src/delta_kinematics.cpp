#include "kinematics_interface_delta/delta_kinematics.hpp"
#include <cmath>
#include <limits>
#include <iostream>

using namespace kinematics_interface_delta;

static constexpr double PI = M_PI;
static constexpr double sqrt3 = 1.7320508075688772; // std::sqrt(3.0)
static constexpr double sin30 = 0.5;

DeltaKinematics::DeltaKinematics(const DeltaGeometry &g, bool debug) : g_(g), debug_(debug) {}

// Helper: calculate angle for one arm given target coords in that arm's coordinate frame
bool DeltaKinematics::delta_calcAngleYZ(double x0, double y0, double z0, double &theta) const
{
  // based on standard delta robot inverse kinematics derivation
  double y1 = -(g_.f - g_.e); // shift for base/end effector radius difference
  // apply motor Z offset to z coordinate (shift workspace if motors are offset)
  z0 -= g_.motor_z_offset;

  // z0 is expected to be negative for positions below the base motors in many conventions
  double a = (x0 * x0 + y0 * y0 + z0 * z0 + g_.rf * g_.rf - g_.re * g_.re - y1 * y1) / (2.0 * z0);
  double b = (y1 - y0) / z0;

  double disc = g_.rf * g_.rf * (b * b + 1) - (a + b * y1) * (a + b * y1);
  if (disc < 0) {
    if (debug_) std::cerr << "delta_calcAngleYZ: disc<0 (no solution). disc=" << disc << " a=" << a << " b=" << b << " y1=" << y1 << "\n";
    return false; // non-existing
  }

  // Try both quadratic roots and pick the one that yields a valid theta.
  double denom = (b * b + 1);
  double sqrt_disc = std::sqrt(disc);
  double yj_candidates[2];
  yj_candidates[0] = (y1 - a * b - sqrt_disc) / denom;
  yj_candidates[1] = (y1 - a * b + sqrt_disc) / denom;

  for (int i = 0; i < 2; ++i) {
    double yj = yj_candidates[i];
    double zj = a + b * yj;
    double theta_cand = std::atan2(-zj, y1 - yj);
    if (debug_) {
      std::cerr << "delta_calcAngleYZ: try " << i << " yj=" << yj << " zj=" << zj << " theta=" << theta_cand << "\n";
    }
    // Basic numerical sanity checks: finite and z not NaN
    if (std::isfinite(theta_cand) && std::isfinite(yj) && std::isfinite(zj)) {
      theta = theta_cand;
      return true;
    }
  }

  if (debug_) std::cerr << "delta_calcAngleYZ: no valid theta from candidates\n";
  return false;
}

std::optional<std::array<double,3>> DeltaKinematics::delta_calcForward(double theta1, double theta2, double theta3) const
{
  // Forward kinematics implementation from:
  // https://hypertriangle.com/~alex/delta-robot-tutorial/
  // This is the proven algorithm by mzavatsky
  
  if (debug_) {
    std::cerr << "FK: geometry e=" << g_.e << " f=" << g_.f << " re=" << g_.re << " rf=" << g_.rf << " motor_z=" << g_.motor_z_offset << "\n";
    std::cerr << "FK: input thetas=[" << theta1 << ", " << theta2 << ", " << theta3 << "]\n";
  }
  
  const double sin30 = 0.5;
  const double tan60 = std::sqrt(3.0);
  
  double t = g_.f - g_.e;  // base radius - EE radius
  
  // Convert angles to elbow joint positions
  double y1 = -(t + g_.rf * std::cos(theta1));
  double z1 = -g_.rf * std::sin(theta1) + g_.motor_z_offset;
  
  double y2 = (t + g_.rf * std::cos(theta2)) * sin30;
  double x2 = y2 * tan60;
  double z2 = -g_.rf * std::sin(theta2) + g_.motor_z_offset;
  
  double y3 = (t + g_.rf * std::cos(theta3)) * sin30;
  double x3 = -y3 * tan60;
  double z3 = -g_.rf * std::sin(theta3) + g_.motor_z_offset;
  
  double dnm = (y2 - y1) * x3 - (y3 - y1) * x2;
  
  double w1 = y1 * y1 + z1 * z1;
  double w2 = x2 * x2 + y2 * y2 + z2 * z2;
  double w3 = x3 * x3 + y3 * y3 + z3 * z3;
  
  // x = (a1*z + b1) / dnm
  double a1 = (z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1);
  double b1 = -((w2 - w1) * (y3 - y1) - (w3 - w1) * (y2 - y1)) / 2.0;
  
  // y = (a2*z + b2) / dnm
  double a2 = -(z2 - z1) * x3 + (z3 - z1) * x2;
  double b2 = ((w2 - w1) * x3 - (w3 - w1) * x2) / 2.0;
  
  // a*z^2 + b*z + c = 0
  double a = a1 * a1 + a2 * a2 + dnm * dnm;
  double b = 2.0 * (a1 * b1 + a2 * (b2 - y1 * dnm) - z1 * dnm * dnm);
  double c = (b2 - y1 * dnm) * (b2 - y1 * dnm) + b1 * b1 + dnm * dnm * (z1 * z1 - g_.re * g_.re);
  
  // Discriminant
  double disc = b * b - 4.0 * a * c;
  if (disc < 0.0) {
    return std::nullopt; // Non-existing position
  }
  
  double z0 = -0.5 * (b + std::sqrt(disc)) / a;
  double x0 = (a1 * z0 + b1) / dnm;
  double y0 = (a2 * z0 + b2) / dnm;
  
  if (debug_) {
    std::cerr << "FK: result position=[" << x0 << ", " << y0 << ", " << z0 << "]\n";
  }
  
  return std::array<double,3>{x0, y0, z0};
}

std::optional<std::array<double,3>> DeltaKinematics::forward(const std::array<double,3> &thetas) const
{
  return delta_calcForward(thetas[0], thetas[1], thetas[2]);
}

std::array<std::array<double,3>, 3> DeltaKinematics::calculate_elbow_positions(const std::array<double,3> &thetas) const
{
  // Calculate elbow positions for each arm based on joint angles
  // These are the positions where upper arm meets lower arm (forearm)
  double t = g_.f - g_.e;  // base radius - EE radius
  
  std::array<std::array<double,3>, 3> elbows;
  
  // Elbow 1 (arm at 0 degrees around base)
  elbows[0][0] = 0.0;  // x
  elbows[0][1] = -(t + g_.rf * std::cos(thetas[0]));  // y
  elbows[0][2] = -g_.rf * std::sin(thetas[0]) + g_.motor_z_offset;  // z (add motor offset)
  
  // Elbow 2 (arm at 120 degrees around base, radial direction at 30° from +X)
  elbows[1][0] = (t + g_.rf * std::cos(thetas[1])) * std::cos(PI/6.0);  // x = cos30 * (...)
  elbows[1][1] = (t + g_.rf * std::cos(thetas[1])) * sin30;  // y = sin30 * (...)
  elbows[1][2] = -g_.rf * std::sin(thetas[1]) + g_.motor_z_offset;  // z
  
  // Elbow 3 (arm at 240 degrees around base, radial direction at 150° from +X)
  elbows[2][0] = -(t + g_.rf * std::cos(thetas[2])) * std::cos(PI/6.0);  // x = -cos30 * (...)
  elbows[2][1] = (t + g_.rf * std::cos(thetas[2])) * sin30;  // y = sin30 * (...)
  elbows[2][2] = -g_.rf * std::sin(thetas[2]) + g_.motor_z_offset;  // z
  
  return elbows;
}

std::optional<std::array<double,3>> DeltaKinematics::inverse(const std::array<double,3> &pos) const
{
  // Use Levenberg-Marquardt style damped least-squares with adaptive lambda
  const double eps = 1e-7;
  const double tol = 1e-6; // meters
  const int max_iter = 200;
  std::array<double,3> theta = {0.0, 0.0, 0.0};

  double lambda = 1e-3;
  double prev_err_norm = 1e12;

  for (int iter = 0; iter < max_iter; ++iter) {
    auto fopt = forward(theta);
    if (!fopt) {
      if (debug_) std::cerr << "inverse: forward failed at iter " << iter << "\n";
      return std::nullopt;
    }
    auto f = *fopt;
    std::array<double,3> err = {f[0]-pos[0], f[1]-pos[1], f[2]-pos[2]};
    double err_norm = std::sqrt(err[0]*err[0] + err[1]*err[1] + err[2]*err[2]);
    if (err_norm < tol) return theta;

    // numeric Jacobian J (3x3)
    double J[3][3];
    for (int j = 0; j < 3; ++j) {
      auto th_eps = theta;
      th_eps[j] += eps;
      auto f2opt = forward(th_eps);
      if (!f2opt) {
        if (debug_) std::cerr << "inverse: forward failed for finite diff at j=" << j << "\n";
        return std::nullopt;
      }
      auto f2 = *f2opt;
      J[0][j] = (f2[0] - f[0]) / eps;
      J[1][j] = (f2[1] - f[1]) / eps;
      J[2][j] = (f2[2] - f[2]) / eps;
    }

    // Build J^T * J (3x3) and J^T * err (3)
    double JTJ[3][3] = {};
    double JTerr[3] = {};
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        double sum = 0.0;
        for (int k = 0; k < 3; ++k) sum += J[k][r] * J[k][c];
        JTJ[r][c] = sum;
      }
      double sum2 = 0.0;
      for (int k = 0; k < 3; ++k) sum2 += J[k][r] * err[k];
      JTerr[r] = sum2;
    }

    // Solve (JTJ + lambda I) dx = -J^T * err with adaptive lambda
    auto det3 = [&](double M[3][3]){
      return M[0][0]*(M[1][1]*M[2][2]-M[1][2]*M[2][1]) -
             M[0][1]*(M[1][0]*M[2][2]-M[1][2]*M[2][0]) +
             M[0][2]*(M[1][0]*M[2][1]-M[1][1]*M[2][0]);
    };

    // Try up to a few lambda adjustments per iteration
    bool accepted = false;
    std::array<double,3> dx = {0.0,0.0,0.0};
    for (int attempt = 0; attempt < 8 && !accepted; ++attempt) {
      double A[3][3];
      for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) A[r][c] = JTJ[r][c] + (r==c ? lambda : 0.0);
      double rhs[3];
      for (int r = 0; r < 3; ++r) rhs[r] = -JTerr[r];

      double detA = det3(A);
      if (std::abs(detA) < 1e-15) {
        if (debug_) std::cerr << "inverse: singular JTJ+lambda at iter "<<iter<<" attempt "<<attempt<<" det="<<detA<<"\n";
        lambda *= 10.0;
        continue;
      }

      double Mx[3][3] = {{rhs[0], A[0][1], A[0][2]}, {rhs[1], A[1][1], A[1][2]}, {rhs[2], A[2][1], A[2][2]}};
      double My[3][3] = {{A[0][0], rhs[0], A[0][2]}, {A[1][0], rhs[1], A[1][2]}, {A[2][0], rhs[2], A[2][2]}};
      double Mz[3][3] = {{A[0][0], A[0][1], rhs[0]}, {A[1][0], A[1][1], rhs[1]}, {A[2][0], A[2][1], rhs[2]}};
      dx[0] = det3(Mx) / detA;
      dx[1] = det3(My) / detA;
      dx[2] = det3(Mz) / detA;

      // trial theta
      std::array<double,3> theta2 = {theta[0]+dx[0], theta[1]+dx[1], theta[2]+dx[2]};
      auto f2opt = forward(theta2);
      if (!f2opt) {
        lambda *= 10.0;
        continue;
      }
      auto f2 = *f2opt;
      std::array<double,3> err2 = {f2[0]-pos[0], f2[1]-pos[1], f2[2]-pos[2]};
      double err2_norm = std::sqrt(err2[0]*err2[0] + err2[1]*err2[1] + err2[2]*err2[2]);

      if (err2_norm < err_norm) {
        // Accept
        theta = theta2;
        prev_err_norm = err_norm;
        err_norm = err2_norm;
        lambda = std::max(1e-12, lambda * 0.1);
        accepted = true;
      } else {
        // Reject, increase damping
        lambda *= 10.0;
      }
    }

    if (!accepted) {
      if (debug_) std::cerr << "inverse: no lambda produced improvement at iter "<<iter<<"\n";
      return std::nullopt;
    }

    if (debug_) std::cerr << "inverse iter="<<iter<<" err_norm="<<err_norm<<" lambda="<<lambda<<" dx="<<dx[0]<<","<<dx[1]<<","<<dx[2]<<"\n";
    if (err_norm < tol) return theta;
  }

  if (debug_) std::cerr << "inverse: did not converge after max_iter" << std::endl;
  return std::nullopt;
}
