#pragma once

#include <tuple>
#include <unordered_map>
#include "Eigen/Dense"

namespace sigma_points {
  constexpr size_t NSigma(size_t Nstate) { return 2*Nstate + 1; }
  constexpr size_t NState(size_t Nsigma) { return (Nsigma - 1) / 2; }
  
  template<size_t Nstate, size_t Nsigma = NSigma(Nstate)>
  using SigmaPoints = Eigen::Matrix<double, Nstate, Nsigma>;

  template<size_t Nstate>
  using State = Eigen::Matrix<double, Nstate, 1>;

  template<size_t Nstate>
  using Covariance = Eigen::Matrix<double, Nstate, Nstate>;

  namespace internal {
    template<size_t Nsigma, typename R = State<Nsigma>>
    R weights(double lambda) {
      static_assert((Nsigma - 1) % 2 == 0, "Invalid size");
      static_assert(std::is_same<R, State<Nsigma>>::value, "Invalid return type");

      static std::unordered_map<double, R> cache;
      const double d = lambda + NState(Nsigma);

      auto cw = cache.find(lambda);
      if(cw != end(cache)) return cw->second;

      auto& w = cache[lambda] = R::Constant(1./(2*d)); 
      w(0) = lambda/d;
      return w;
    }
  }

  template<typename S, typename U,
           int NSstate = S::RowsAtCompileTime, int NSsigma = S::ColsAtCompileTime,
           int NUstate = U::RowsAtCompileTime, int NUsigma = U::ColsAtCompileTime>
  Eigen::Matrix<double, NSstate, NUstate>
  xCorrelation(const S& Sc, const U& Uc, double lambda = 3. - NState(NSsigma)) {
    static_assert(NSstate == S::RowsAtCompileTime, "Invalid rows count S");
    static_assert(NSsigma == S::ColsAtCompileTime, "Invalid cols count S");
    static_assert(NUstate == U::RowsAtCompileTime, "Invalid rows count U");
    static_assert(NUsigma == U::ColsAtCompileTime, "Invalid cols count U");
    static_assert(NSsigma == NUsigma, "Cols count mismatch");
    static_assert((NSsigma - 1) % 2 == 0, "Invalid number of columns");

    const auto w = internal::weights<NSsigma>(lambda);
    return Sc * (Uc.transpose().array().colwise() * w.array()).matrix();
  }
   
  template<typename S, int Nstate = S::RowsAtCompileTime, int Nsigma = NSigma(Nstate)>
  SigmaPoints<Nstate, Nsigma>
  create(const S& x, const Covariance<Nstate>& P, double lambda = 3. - Nstate) {
    static_assert(std::is_same<S, State<Nstate>>::value, "Invalid state");
    static_assert(Nstate == S::RowsAtCompileTime, "Invalid state size");
    static_assert(Nsigma == 2*Nstate + 1, "Invalid number of sigma points");

    Covariance<Nstate> A = P.llt().matrixL();
    A *= sqrt(Nstate + lambda);

    SigmaPoints<Nstate, Nsigma> sp;
    sp.col(0) = x;
    sp.template block<Nstate, Nstate>(0, 1 + 0) = A.colwise() + x;
    sp.template block<Nstate, Nstate>(0, 1 + Nstate) = (-A).colwise() + x;
    return sp;
  };

  template<typename S, int Nstate = S::RowsAtCompileTime, int Nsigma = S::ColsAtCompileTime>
  std::tuple<State<Nstate>, Covariance<Nstate>>
  predict(const S& SP, double lambda = 3. - NState(Nsigma)) {
    static_assert(Nstate == S::RowsAtCompileTime, "Invalid rows count");
    static_assert(Nsigma == S::ColsAtCompileTime, "Invalid cols count");
    static_assert((Nsigma - 1) % 2 == 0, "Invalid number of columns");
    
    const auto w = internal::weights<Nsigma>(lambda);
    auto x = SP * w;
    auto SPc = SP.colwise() - x;
    return std::make_tuple(std::move(x), xCorrelation(SPc, SPc));
  }
}
