#pragma once

#include <tuple>
#include <chrono>
#include "sigma_points.h"

#include <iostream>

namespace kalman_filter {
  template<typename Model>
  using State = std::tuple<bool, typename Model::State, double>;
  
  namespace internal {
    template<typename Model>
    using Prediction = std::tuple<std::chrono::microseconds,
                                  Eigen::Matrix<double, Model::N,
                                                        sigma_points::NSigma(Model::N_A)>>;
    
    template<typename Model>
    using Update = std::tuple<typename Model::State, double>;
    
    template<typename Model>
    Prediction<Model> predict(std::chrono::microseconds now, typename Model::State state) {
      const auto& ts = std::get<0>(state);
      const auto& x = Model::Augment(std::get<1>(state));
      const auto& P = Model::Augment(std::get<2>(state));
      
      auto dt = std::chrono::duration_cast<typename Model::Interval>(now - ts);
      auto SP = Model::F(sigma_points::create(x, P), dt);

      return Prediction<Model>{now, SP};
    }

    template<typename Model, typename Z>
    Update<Model> update(typename Z::Measurement z, Prediction<Model> state) {
      const auto& now = std::get<0>(state);
      const auto& SP = std::get<1>(state);

      auto ZP = Z::H(SP);
      auto Zp = sigma_points::predict(ZP);
      const auto& zp = std::get<0>(Zp);
      const auto S = std::get<1>(Zp) + Z::R;
      const auto Sinv = S.inverse();
      
      auto Xp = sigma_points::predict(SP);
      const auto& x = std::get<0>(Xp);
      const auto& P = std::get<1>(Xp);

      auto T = sigma_points::xCorrelation(SP.colwise() - x, ZP.colwise() - zp);
      auto K = T*Sinv;

      const auto dz = Z::Normalize(z - zp);
      auto e = dz.transpose() * Sinv * dz;

      return Update<Model>{typename Model::State{now, x + K*dz, P - K*S*K.transpose()}, e};
    }
  }
  
  template<typename Model, typename Z>
  State<Model> update(typename Z::Package package, State<Model> state) {
    auto now = std::get<0>(package);
    auto z = std::get<1>(package);

    auto initialized = std::get<0>(state);
    auto s = std::get<1>(state);
    
    if(initialized) {
      auto p = internal::predict<Model>(now, std::move(s));
      auto u = internal::update<Model, Z>(std::move(z), std::move(p));
      return State<Model>{true, std::get<0>(u), std::get<1>(u)};
    } else {
      return State<Model>{true, Model::template Init<Z>(now, std::move(z)), 0};
    }
  }
}
