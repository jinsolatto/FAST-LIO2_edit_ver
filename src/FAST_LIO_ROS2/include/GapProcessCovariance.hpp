#pragma once
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <cmath>
#include <stdexcept>
#include <string>

// Opt-in completed-gap covariance candidate; also used by offline validation.
// Continuous densities: angular acceleration rad^2/s^3; acceleration m^2/s^3.
// Local linear, fixed-frame hold model. Exact initial omega is assumed.
// Not an endpoint-conditioned trapezoidal error covariance, nor native Q.
namespace gap_covariance {
using Mat3 = Eigen::Matrix3d;
using Mat9 = Eigen::Matrix<double, 9, 9>; // theta tangent, world velocity, world position
using Mat23 = Eigen::Matrix<double, 23, 23>;
inline void checkDensity(const Mat3& q) {
  if (!q.allFinite() || !q.isApprox(q.transpose(), 1e-12))
    throw std::invalid_argument("Density must be finite symmetric PSD");
  Eigen::SelfAdjointEigenSolver<Mat3> es(q);
  if (es.eigenvalues().minCoeff() < 0.)
    throw std::invalid_argument("Negative spectral density");
}
inline Mat9 discrete(double dt, const Mat3& angular_accel_density_tangent,
                     const Mat3& accel_density_world) {
  if (!std::isfinite(dt) || dt < 0.) throw std::invalid_argument("Invalid gap duration");
  checkDensity(angular_accel_density_tangent); checkDensity(accel_density_world);
  const double d2=dt*dt, d3=d2*dt;
  Mat9 p=Mat9::Zero();
  p.block<3,3>(0,0)=angular_accel_density_tangent*(d3/3.);
  p.block<3,3>(3,3)=accel_density_world*dt;
  p.block<3,3>(6,6)=accel_density_world*(d3/3.);
  p.block<3,3>(3,6)=accel_density_world*(d2/2.);
  p.block<3,3>(6,3)=p.block<3,3>(3,6).transpose();
  if (!p.allFinite()) throw std::overflow_error("Covariance overflow");
  return p;
}
// Diagnostic block ablations, each preserving PSD. Full is the original path.
inline Mat9 selectBlocks(const Mat9& p, const std::string& mode) {
  if(mode=="full") return p;
  Mat9 out=Mat9::Zero();
  if(mode=="rotation")out.block<3,3>(0,0)=p.block<3,3>(0,0);
  else if(mode=="velocity")out.block<3,3>(3,3)=p.block<3,3>(3,3);
  else if(mode=="position")out.block<3,3>(6,6)=p.block<3,3>(6,6);
  else if(mode=="velocity_position")out.block<6,6>(3,3)=p.block<6,6>(3,3);
  else if(mode=="no_cross")for(int i=0;i<3;++i)out.block<3,3>(3*i,3*i)=p.block<3,3>(3*i,3*i);
  else throw std::invalid_argument("Unknown gap block mode");
  return out;
}
// Verified against use-ikfom.hpp: pos=0, rot=3, vel=12, 23 tangent DOF.
// No estimator pointer, no change_P(), and no hidden timing state.
inline Mat23 embedFastLio(const Mat9& p) {
  Mat23 out=Mat23::Zero();const int index[3]={3,12,0};
  for(int i=0;i<3;++i)for(int j=0;j<3;++j)
    out.block<3,3>(index[i],index[j])=p.block<3,3>(3*i,3*j);
  return out;
}
} // namespace gap_covariance
