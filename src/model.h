#pragma once

#include <tuple>
#include <chrono>

#include "Eigen/Dense"

struct Model {
  // Model state fields description
  enum Field{
    X = 0,
    Y,
    C, // number of cartesian fileds

    VX = C,
    VY,
    N, // number of fields
  };

  using Interval = std::chrono::duration<double>; // measurement interval

  using x = Eigen::Matrix<double, N, 1>;  // model state vector
  using P = Eigen::Matrix<double, N, N>;  // model P matrix

  using Cartesian = Eigen::Matrix<double, Model::C, 1>; // cartesian part of the state

  static const Eigen::Matrix<double, C, C> Q; // model movement noise 
  static const P I; // model identity matrix

  static P F(Interval dt);
  static Eigen::Matrix<double, N, 2> G(Interval dt);

  using State = std::tuple<std::chrono::microseconds, x, P>; // model complete state

  template<typename Z>
  static State Init(std::chrono::microseconds now, typename Z::Measurement z) {
    return std::make_tuple(now,                      // initial timestamp
                           // initial state 
                           (x() << Z::Cartesian(z),  // measurement mapped to cartesian coordinates
                                   Eigen::Matrix<double, N-C, 1>::Zero() // zeros for remaining fields
                           ).finished(),
                           // R | 0
                           // 0 | Q
                           // R - measurement covariance, Q - process covariance (acceleration)
                           (P() << Z::R.template block<C, C>(0, 0), Eigen::Matrix<double, C, N-C>::Zero(),
                                   Eigen::Matrix<double, N-C, C>::Zero(), Q
                           ).finished());
  }
};

template<typename IS, typename D>
IS& operator>> (IS& is, Eigen::MatrixBase<D>& v) {
  for(auto i = 0; i < v.rows(); ++i) {
    for(auto j = 0; j < v.cols(); ++j) {
      is >> v(i, j);
    }
  }
  return is;
}
