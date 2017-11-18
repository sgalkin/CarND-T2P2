#include "laser.h"

namespace {
  constexpr double px_std = 0.15;
  constexpr double py_std = 0.15;
}

const Laser::TAG Laser::Tag = 'L';

const Laser::Covariance Laser::R = []{
  return (Laser::Covariance()
          << px_std*px_std,             0,
                         0, py_std*py_std
         ).finished();
}();

Laser::Measurement Laser::Normalize(Laser::Measurement z) {
  return z;
}
  
Model::Cartesian Laser::Cartesian(const Laser::Measurement& z) {
  return z;
}
