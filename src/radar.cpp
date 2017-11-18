#include "radar.h"

#include <cmath>
#include "tools.h"

namespace {
  constexpr double ro_std = 0.3;
  constexpr double phi_std = 0.03;
  constexpr double phi_dot_std = 0.3;
}

const Radar::TAG Radar::Tag = 'R';

const Radar::Covariance Radar::R = []{
  return (Radar::Covariance()
          << ro_std*ro_std,               0,                       0,
                         0, phi_std*phi_std,                       0,
                         0,               0, phi_dot_std*phi_dot_std
         ).finished();
}();

Radar::Measurement Radar::Normalize(Radar::Measurement z) {
  z(1) = tools::NormalizeAngle(z(1));
  return z;
}
  
Model::Cartesian Radar::Cartesian(const Radar::Measurement& z) {
  auto t = tan(z(1));
  auto d = sqrt(1 + pow(t, 2));
  auto x = z(0) / d;
  return (Model::Cartesian() << x*t, x).finished();
}
