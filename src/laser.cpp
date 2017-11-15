#include "laser.h"

const Laser::TAG Laser::Tag = 'L';

const Laser::Covariance Laser::R = []{
  return (Laser::Covariance() << 0.0225, 0,
                                 0, 0.0225).finished();
}();

Laser::Measurement Laser::Hp(const Model::x& x) {
  return Hj(x) * x;
}

Laser::Projection Laser::Hj(const Model::x&) {
  static const Projection H = [] {
    return (Projection() << 1, 0, 0, 0,
                            0, 1, 0, 0).finished();
  }();
  return H;
}

Laser::Measurement Laser::Normalize(Laser::Measurement z) {
  return z;
}
  
Model::Cartesian Laser::Cartesian(const Laser::Measurement& z) {
  return z;
}
