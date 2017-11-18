#include "model.h"

namespace {
  const double std_a = 0.5;
  const double std_psi_dot_dot = 0.25;
}

const Eigen::Matrix<double, Model::N_A - Model::N,
                            Model::N_A - Model::N> Model::Q = []{
  return (Eigen::Matrix<double, N_A - N, N_A - N>()
          << std_a*std_a, 0,
             0, std_psi_dot_dot*std_psi_dot_dot
         ).finished();
}();

Model::xa Model::Augment(const Model::x& x) {
  return (Model::xa() << x,
                         Eigen::Matrix<double, Model::N_A - Model::N, 1>::Zero()
         ).finished();
}

Model::Pa Model::Augment(const Model::P& P) {
  Model::Pa r{Model::Pa::Zero()};
  r.topLeftCorner<Model::N, Model::N>() = P;
  r.bottomRightCorner<Model::N_A - Model::N,
                      Model::N_A - Model::N>() = Q;
  return r;
}
