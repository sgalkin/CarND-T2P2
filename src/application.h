#pragma once

#include <string>
#include <exception>
#include <vector>

#include "model.h"
#include "tools.h"
#include "filter.h"
#include "measurement.h"

using State = std::tuple<bool, Model::State>;
using MeasurementFilter = Filter<MeasurementBase::TAG, Deny>;

State update(std::istringstream& iss, State state);

template<typename Protocol>
class Application {
public:
  explicit Application(MeasurementFilter filter) :
    state_{false, Model::State()},
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
    if(!filter_(iss.peek())) { // Ignores filtered measurement
      return Protocol::formatResponse();
    }

    state_ = update(iss, state_);
    estimations_.push_back(std::get<1>(std::get<1>(state_)));

    Model::x gt;
    iss >> gt;
    truth_.emplace_back(std::move(gt));

    auto rmse = tools::RMSE(estimations_, truth_);
    return Protocol::formatResponse(estimations_.back(), rmse);
  }

private:
  State state_;
  MeasurementFilter filter_;
  
  // used to compute the RMSE
  std::vector<Model::x> estimations_;
  std::vector<Model::x> truth_;
};
