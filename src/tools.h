#pragma once

#include <vector>
#include <numeric>
#include <exception>

namespace tools {
  // A helper function to calculate RMSE.
  template<typename V>
  V RMSE(const std::vector<V>& estimations, const std::vector<V>& ground_truth) {
    if(ground_truth.empty() || ground_truth.size() != estimations.size()) {
      throw std::invalid_argument("size mismatch for RMSE");
    }

    const auto& f = ground_truth.front();
    auto v = std::inner_product(begin(estimations), end(estimations),
                                begin(ground_truth),
                                V(V::Zero(f.rows(), f.cols())),
                                [](const V& acc, const V& v) {
                                  return acc + v;
                                },
                                [](const V& e, const V& gt) {
                                  return (e - gt).array().square().matrix();
                                });
    return (v / ground_truth.size()).array().sqrt().matrix();
  }
}
