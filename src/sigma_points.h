#pragma once

#include <tuple>
#include <unordered_map>
#include "Eigen/Dense"

namespace sigma_points {
  constexpr size_t NSigma(size_t Nstate) { return 2*Nstate + 1; }
  constexpr size_t NState(size_t Nsigma) { return (Nsigma - 1) / 2; }
  
  template<size_t Nstate, size_t Nsigma>
  using SigmaPoints = Eigen::Matrix<double, Nstate, Nsigma>;

  template<size_t Nstate, size_t Nsigma = NSigma(Nstate)>
  using SigmaPackage = std::tuple<SigmaPoints<Nstate, Nsigma>, double>;

  template<size_t Nstate>
  using State = Eigen::Matrix<double, Nstate, 1>;

  template<size_t Nstate>
  using Covariance = Eigen::Matrix<double, Nstate, Nstate>;

  template<size_t Nstate>
  using ModelPackage = std::tuple<State<Nstate>, Covariance<Nstate>>;

  namespace internal {
    template<size_t Nsigma, typename R = Eigen::Matrix<double, Nsigma, 1>>
    R weights(double lambda) {
      static_assert((Nsigma - 1) % 2 == 0,
                    "Invalid size");
      static_assert(std::is_same<R, Eigen::Matrix<double, Nsigma, 1>>::value,
                    "Invalid return type");

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
           typename R = Eigen::Matrix<double, S::RowsAtCompileTime,
                                              U::RowsAtCompileTime>>
  R xCorrelation(const S& Sc, const U& Uc, double lambda) {
    static_assert(int(S::ColsAtCompileTime) == int(U::ColsAtCompileTime),
                  "Cols count mismatch");
    static_assert((S::ColsAtCompileTime - 1) % 2 == 0,
                  "Invalid number of columns");
    static_assert(std::is_same<R,
                               Eigen::Matrix<double, S::RowsAtCompileTime,
                                                     U::RowsAtCompileTime>>::value,
                  "Invalid return type");

    const auto w = internal::weights<S::ColsAtCompileTime>(lambda);
    return Sc * (Uc.transpose().array().colwise() * w.array()).matrix();
  }
   
  template<typename S, int Nstate = S::RowsAtCompileTime, int Nsigma = NSigma(Nstate)>
  SigmaPackage<Nstate, Nsigma>
  create(const S& x, const Covariance<Nstate>& P, double lambda = 3. - Nstate) {
    static_assert(std::is_same<S, State<Nstate>>::value, "Invalid state");
    static_assert(Nstate == S::RowsAtCompileTime, "Invalid state size");
    static_assert(Nsigma == NSigma(Nstate), "Invalid number of sigma points");

    Covariance<Nstate> A = P.llt().matrixL();
    A *= sqrt(Nstate + lambda);

    SigmaPoints<Nstate, Nsigma> sp;
    sp.col(0) = x;
    sp.template block<Nstate, Nstate>(0, 1 + 0) = A.colwise() + x;
    sp.template block<Nstate, Nstate>(0, 1 + Nstate) = (-A).colwise() + x;
    return SigmaPackage<Nstate, Nsigma>{std::move(sp), lambda};
  };

  template<typename S,
           size_t Nstate = std::tuple_element<0, S>::type::RowsAtCompileTime>
  ModelPackage<Nstate> predict(const S& package) {
    const auto& SP = std::get<0>(package);
    const auto lambda = std::get<1>(package);
    
    const auto w = internal::weights<std::tuple_element<0, S>::type::ColsAtCompileTime>(lambda);
    auto x = SP * w;
    auto SPc = SP.colwise() - x;
    return ModelPackage<Nstate>{std::move(x), xCorrelation(SPc, SPc, lambda)};
  }
}
