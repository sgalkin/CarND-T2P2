#pragma once

#include <tuple>
#include <chrono>

namespace kalman_filter {
  template<typename Model>
  using State = typename Model::State;
  
  namespace internal {
    template<typename Model>
    State<Model> predict(std::chrono::microseconds now, State<Model> state) {
      const auto& ts = std::get<0>(state);
      const auto& x = std::get<1>(state);
      const auto& P = std::get<2>(state);
      
      auto dt = std::chrono::duration_cast<typename Model::Interval>(now - ts);
      auto F = Model::F(dt);
      auto G = Model::G(dt);
      
      return std::make_tuple(now, F*x, F*P*F.transpose() + G*Model::Q*G.transpose());
    }

    template<typename Model, typename Z>
    State<Model> update(typename Z::Measurement z, State<Model> state) {
      const auto& now = std::get<0>(state);
      const auto& x = std::get<1>(state);
      const auto& P = std::get<2>(state);

      auto Hj = Z::Hj(x);
      auto zp = Z::Hp(x); 
      auto PHjt = P * Hj.transpose();
      
      auto y = Z::Normalize(z - zp);
      auto S = Hj * PHjt + Z::R;
      auto K = PHjt * S.inverse();
      
      return std::make_tuple(now, x + K*y, (Model::I - K*Hj)*P);
    }
  }
  
  template<typename Model, typename Z>
  std::tuple<bool, State<Model>>
  update(typename Z::Package package, std::tuple<bool, State<Model>> state) {
    auto now = std::get<0>(package);
    auto z = std::get<1>(package);

    auto initialized = std::get<0>(state);
    auto s = std::get<1>(state);
    
    if(initialized) {
      // don't predict if measuremnet came for the same moment as previous one
      auto p = std::get<0>(s) == now ? s : internal::predict<Model>(now, std::move(s));
      auto u = internal::update<Model, Z>(std::move(z), std::move(p));
      return std::make_tuple(true, u);
    } else {
      return std::make_tuple(true, Model::template Init<Z>(now, std::move(z)));
    }
  }
}
