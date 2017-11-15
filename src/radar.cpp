#include "radar.h"
#include <cmath>

namespace {
  constexpr double epsilon = 1e-8;
}

const Radar::TAG Radar::Tag = 'R';

const Radar::Covariance Radar::R = []{
  return (Radar::Covariance() << 0.09, 0, 0,
                                 0, 0.0009, 0,
                                 0, 0, 0.09).finished();
}();

Radar::Measurement Radar::Hp(const Model::x& x) {
  auto ro = sqrt(x.block<2, 1>(0, 0).array().square().sum());
  if(ro < epsilon) ro = epsilon;
  auto phi = atan2(x(1), x(0));
  auto ro_dot = (x.block<2, 1>(0, 0).array() * x.block<2, 1>(2, 0).array()).sum() / ro;
  return (Measurement() << ro, phi, ro_dot).finished();
}

Radar::Projection Radar::Hj(const Model::x& x) {
  auto ro2 = x.block<2, 1>(0, 0).array().square().sum();
  if(ro2 < epsilon) ro2 = epsilon;
  auto ro = sqrt(ro2);
  auto ro3 = ro * ro2;
  auto p = x(0)*x(2) - x(1)*x(3);
  return (Projection() << x(0)/ro, x(1)/ro, 0, 0, 
                          -x(3)/ro2, x(2)/ro2, 0, 0, 
                          -x(3)*p/ro3, x(2)*p/ro3, x(0)/ro, x(1)/ro).finished();
}

Radar::Measurement Radar::Normalize(Radar::Measurement z) {
  while(z(1) < -M_PI) z(1) += 2*M_PI;
  while(z(1) > M_PI) z(1) -= 2*M_PI;
  return z;
}
  
Model::Cartesian Radar::Cartesian(const Radar::Measurement& z) {
  auto t = tan(z(1));
  auto d = sqrt(1 + pow(t, 2));
  auto x = z(0) / d;
  return (Model::Cartesian() << x*t, x).finished();
}
