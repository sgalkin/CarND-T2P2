#pragma once

#include "Eigen/Dense"
#include "model.h"

struct MeasurementBase {
  using TAG = char;

  template<typename M>
  using Package = std::tuple<std::chrono::microseconds, M>;
};

template<size_t S>
struct MeasurementS : MeasurementBase
{
  static constexpr size_t Size = S;
  
  using Measurement = Eigen::Matrix<double, Size, 1>;
  using Covariance = Eigen::Matrix<double, Size, Size>;
  using Projection = Eigen::Matrix<double, Size, Model::N>;

  using Package = MeasurementBase::Package<Measurement>;
};

template<typename IS, typename M>
IS& operator>> (IS& is, MeasurementBase::Package<M>& p) {
  long long ts = 0;
  is >> std::get<1>(p) >> ts;
  std::get<0>(p) = std::chrono::microseconds(ts);
  return is;
}
