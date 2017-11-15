#include "model.h"

namespace {
  const double noise_ax = 9;
  const double noise_ay = 9;
}

const Eigen::Matrix<double, Model::C, Model::C> Model::Q = []{
  return (Eigen::Matrix<double, C, C>() << noise_ax, 0,
                                           0, noise_ay).finished();
}();

const Model::P Model::I = Model::P::Identity();

Model::P Model::F(Model::Interval dt) {
  auto f = I;
  f(0, 2) = f(1, 3) = dt.count();
  return f;
}

Eigen::Matrix<double, Model::N, 2> Model::G(Model::Interval dt) {
  static const auto I = Eigen::Matrix2d::Identity();
  return (Eigen::Matrix<double, Model::N, 2>() << pow(dt.count(), 2)/2.*I,
                                                  dt.count()*I).finished();
}
