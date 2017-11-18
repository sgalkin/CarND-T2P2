#pragma once

#include "model.h"
#include "measurement.h"

struct Laser : MeasurementS<2> {
  static const TAG Tag;
  static const Covariance R;   // measurement covariance matrix

  template<typename S>
  static Projection<S>H(const S& X) {  // Measurement projection
    return X.template topLeftCorner<Size, S::ColsAtCompileTime>();
  }
  static Measurement Normalize(Measurement z); // Normalization (noop)
  static Model::Cartesian Cartesian(const Measurement& z); // Cartezian tranformation (noop)
};
