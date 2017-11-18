#pragma once

#include <tuple>
#include <chrono>

#include "Eigen/Dense"

struct Model {
  // Model state fields description
  enum Field{
    X = 0,
    Y,
    N_C, // number of cartesian fileds

    V = N_C,
    PSI,
    PSI_DOT,
    N, // number of state fields

    NU_A = N,
    NU_PSI_DOT_DOT,
    N_A // number of augmentation state fields
  };

  using Interval = std::chrono::duration<double>; // measurement interval

  using x = Eigen::Matrix<double, N, 1>;  // model state vector
  using P = Eigen::Matrix<double, N, N>;  // model P matrix

  using xa = Eigen::Matrix<double, N_A, 1>;   // augmented model state vector
  using Pa = Eigen::Matrix<double, N_A, N_A>; // augmented model P matrix
  
  using Cartesian = Eigen::Matrix<double, N_C, 1>; // cartesian part of the state
  
  using State = std::tuple<std::chrono::microseconds, x, P>; // model complete state

  static const Eigen::Matrix<double, N_A - N, N_A - N> Q; // process noise
  
  template<typename Z>
  static State Init(std::chrono::microseconds now, typename Z::Measurement z) {
    return std::make_tuple(now, InitState<Z>(std::move(z)), InitCovariance<Z>());
  }

  static xa Augment(const x& x);
  static Pa Augment(const P& P);

  template<typename A>
  static Eigen::Matrix<double, N, A::ColsAtCompileTime>
  F(const A& x, Interval duration) {
    static_assert(A::RowsAtCompileTime == int(N_A), "Invalid input dimention");

    double dt = duration.count();
    double dt2 = dt * dt / 2;

    Eigen::Matrix<double, 3, A::ColsAtCompileTime> angle;
    angle.row(0) = (2*x.row(PSI) + dt*x.row(PSI_DOT)) / 2.;
    angle.row(1) = angle.row(0).array().cos();
    angle.row(2) = angle.row(0).array().sin();

    Eigen::Matrix<double, 1, A::ColsAtCompileTime> limit{x.row(PSI_DOT) * dt / 2.};
     // Due to limit sin(x) / x definition
    limit = limit.unaryExpr([](double psi){ return psi == 0 ? 1 : sin(psi)/psi; });
    
    Eigen::Matrix<double, N, A::ColsAtCompileTime> r{
      x.template topLeftCorner<N, A::ColsAtCompileTime>()
    };
    r.row(X) += (dt2 * x.row(PSI).array().cos() * x.row(NU_A).array() +
                 dt * x.row(V).array() * angle.row(1).array() * limit.row(0).array()).matrix();
    r.row(Y) += (dt2 * x.row(PSI).array().sin() * x.row(NU_A).array() +
                 dt * x.row(V).array() * angle.row(2).array() * limit.row(0).array()).matrix();
    r.row(V) += dt * x.row(NU_A); // + 0
    r.row(PSI) += dt2 * x.row(NU_PSI_DOT_DOT) + dt * x.row(PSI_DOT);
    r.row(PSI_DOT) += dt * x.row(NU_PSI_DOT_DOT); // + 0
    return r;
  }

private:
  template<typename Z>
  static x InitState(typename Z::Measurement z) {
    return (x() << Z::Cartesian(z),  // measurement mapped to cartesian coordinates,
                   Eigen::Matrix<double, N - N_C, 1>::Zero() // zeros for remaining fields
           ).finished();
  }

  template<typename Z>
  static P InitCovariance() {
    // R | 0
    // 0 | I?
    // R - measurement covariance, Q - process covariance (acceleration)
    P P{P::Zero()};
    P.template topLeftCorner<N_C, N_C>() = Z::R.template topLeftCorner<N_C, N_C>();
    P.template bottomRightCorner<N-N_C, N-N_C>().setIdentity();
    return P;
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
