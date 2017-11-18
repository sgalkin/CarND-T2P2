#pragma once

#include <string>
#include <vector>
#include <unordered_map>

#include "truth.h"
#include "model.h"
#include "measurement.h"

struct WSProtocol {
  static bool checkHeader(const std::string& message);

  static std::string getPayload(std::string message);
  static std::string getMeasurement(std::string payload);

  static std::string formatResponse();
  static std::string formatResponse(const Model::x& estimate,
                                    const Truth::x& RMSE,
                                    const std::unordered_map<MeasurementBase::TAG,
                                                             std::vector<double>>& NIS);
};
