#include "protocol.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

bool WSProtocol::checkHeader(const std::string& message) {
  // "42" at the start of the message means there's a websocket message event.
  //  The 4 signifies a websocket message
  // The 2 signifies a websocket event
  return message.length() > 2 && strncmp(message.data(), "42", 2) == 0;
}

std::string WSProtocol::getPayload(std::string s) {
  // Checks if the SocketIO event has JSON data.
  // If there is data the JSON object in string format will be returned,
  // else the empty string "" will be returned.
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos &&
           b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

std::string WSProtocol::getMeasurement(std::string s) {
  auto j = json::parse(s);
  if (j[0].get<std::string>() != "telemetry") {
    throw std::runtime_error("Unexpected event type");
  }
  
  // j[1] is the data JSON object
  return j[1]["sensor_measurement"];
}
  
std::string WSProtocol::formatResponse() {
  static const std::string manual = "42[\"manual\",{}]";
  return manual;
}
  
std::string WSProtocol::formatResponse(const Model::x& estimate,
                                       const Truth::x& RMSE,
                                       const std::unordered_map<MeasurementBase::TAG,
                                                                std::vector<double>>&) {
  json msgJson;
  msgJson["estimate_x"] = estimate(Model::X);
  msgJson["estimate_y"] = estimate(Model::Y);
  msgJson["rmse_x"] = RMSE(Truth::X);
  msgJson["rmse_y"] = RMSE(Truth::Y);
  msgJson["rmse_vx"] = RMSE(Truth::VX);
  msgJson["rmse_vy"] = RMSE(Truth::VY);
  
  return "42[\"estimate_marker\"," + msgJson.dump() + "]";
}

