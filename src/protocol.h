#pragma once

#include <string>
#include "model.h"

struct WSProtocol {
  static bool checkHeader(const std::string& message);

  static std::string getPayload(std::string message);
  static std::string getMeasurement(std::string payload);

  static std::string formatResponse();
  static std::string formatResponse(const Model::x& estimate, const Model::x& RMSE);
};
