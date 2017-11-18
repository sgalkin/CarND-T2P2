#pragma once

#include "model.h"
#include "measurement.h"

struct Radar : MeasurementS<3> {
  static const TAG Tag;
  static const Covariance R;   // Measurement covariance matrix

  template<typename S>
  static Projection<S> H(const S& X) { // Measurement projection
    Projection<S> z;
    // sqrt(x^2 + y^2)
    z.row(RO) = (X.row(Model::X).array().square() +
                 X.row(Model::Y).array().square()).sqrt()
      .unaryExpr([](double ro) { return ro < epsilon ? epsilon : ro; });
    // atan(y/x)
    z.row(PHI) = X.row(Model::Y).binaryExpr(X.row(Model::X),
                                            [](double y, double x) {
                                              return atan2(y, x);
                                            });
    // v * (x * cos(psi) + y * sin(psi)) / sqrt(x^2 + y^2)
    z.row(RO_DOT) = z.row(RO).array().inverse() * X.row(Model::V).array() * 
      (X.row(Model::X).array() * X.row(Model::PSI).array().cos() +
       X.row(Model::Y).array() * X.row(Model::PSI).array().sin());
    return z;
  }

  static Measurement Normalize(Measurement z); // Normalization
  static Model::Cartesian Cartesian(const Measurement& z);

private:
  static constexpr double epsilon = 1e-6;
  enum {
    RO = 0,
    PHI,
    RO_DOT
  };
};
