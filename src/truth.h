#pragma once

#include "Eigen/Dense"
#include "model.h"

struct Truth {
  enum {
    X,
    Y,
    VX,
    VY,
    PSI,
    PSI_DOT,
    N
  };

  using x = Eigen::Matrix<double, N, 1>;
};

inline Truth::x cast(const Model::x& x) {
  Truth::x t;
  t(Truth::X) = x(Model::X);
  t(Truth::Y) = x(Model::Y);
  t(Truth::VX) = x(Model::V)*cos(x(Model::PSI));
  t(Truth::VY) = x(Model::V)*sin(x(Model::PSI));
  t(Truth::PSI) = x(Model::PSI);
  t(Truth::PSI_DOT) = x(Model::PSI_DOT);
  return t;
}
