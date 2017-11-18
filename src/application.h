#pragma once

#include <string>
#include <exception>
#include <vector>
#include <unordered_map>

#include "model.h"
#include "tools.h"
#include "filter.h"
#include "measurement.h"
#include "truth.h"
#include "kalman_filter.h"

using MeasurementFilter = Filter<MeasurementBase::TAG, Deny>;
using State = kalman_filter::State<Model>;

State update(MeasurementBase::TAG tag, std::istringstream& iss, State state);

template<typename Protocol>
class Application {
public:
  explicit Application(MeasurementFilter filter) :
    state_{false, Model::State(), 0},
    filter_{std::move(filter)} {}
  
  std::string ProcessMessage(std::string message) {
    if (!Protocol::checkHeader(message)) {
      throw std::runtime_error("Unexpected message header");  
    }
    return ProcessPayload(Protocol::getPayload(std::move(message)));
  }

private:
  std::string ProcessPayload(std::string payload) {
    if(payload.empty()) {
      return Protocol::formatResponse();
    }
    return ProcessMeasurement(Protocol::getMeasurement(std::move(payload)));
  }
  
  std::string ProcessMeasurement(std::string measurement) {
    std::istringstream iss(std::move(measurement));
    MeasurementBase::TAG tag{0};
    iss >> tag;
    if(!filter_(tag)) { // Ignores filtered measurement
      return Protocol::formatResponse();
    }

    state_ = update(tag, iss, state_);
    const auto& estimation = std::get<1>(std::get<1>(state_));
    estimations_.push_back(cast(estimation));
    
    NIS_[tag].push_back(std::get<2>(state_));

    Truth::x gt;
    iss >> gt;
    truth_.emplace_back(std::move(gt));

    auto rmse = tools::RMSE(estimations_, truth_);
    return Protocol::formatResponse(estimation, rmse, NIS_);
  }

private:
  State state_;
  MeasurementFilter filter_;
  
  // used to compute the RMSE
  std::vector<Truth::x> estimations_;
  std::vector<Truth::x> truth_;

  // used to evaluate consistency
  std::unordered_map<MeasurementBase::TAG, std::vector<double>> NIS_;
};
