#include <fstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "catch.hpp"
#include "../src/application.h"
#include "../src/laser.h"
#include "../src/radar.h"
#include "../src/model.h"
#include "../src/truth.h"
#include "../src/filter.h"

namespace {
  using NIS = std::unordered_map<MeasurementBase::TAG, std::vector<double>>;
  
  const std::string input = "../data/obj_pose-laser-radar-synthetic-input.txt";

  Truth::x rmse;
  NIS nis;

  struct FileProtocol {
    static bool checkHeader(const std::string&) { return true; }

    static std::string getPayload(std::string message) { return message; }
    static std::string getMeasurement(std::string payload) { return payload; }

    static std::string formatResponse() { return std::string(); }
    static std::string formatResponse(const Model::x&, const Truth::x& RMSE, const NIS& NIS) {
      rmse = RMSE;
      nis = NIS;
      return std::string();
    }
  };

  Truth::x RMSE_EXPECTATION = []{
    return (Truth::x() << .09,
                          .10,
                          .40,
                          .30,
                          std::numeric_limits<double>::infinity(),
                          std::numeric_limits<double>::infinity()
           ).finished();
  }();
}

TEST_CASE("Rubric RMSE", "[require]") {
  std::ifstream f(input.c_str());
  Application<FileProtocol> app(MeasurementFilter().add(Laser::Tag).add(Radar::Tag));
  for(std::string s; std::getline(f, s); ) {
    app.ProcessMessage(s);
  }

  INFO([]{
      Eigen::MatrixXd report(rmse.size(), 3);
      report.col(0) = rmse;
      report.col(1) = RMSE_EXPECTATION;
      report.col(2) = rmse - RMSE_EXPECTATION;
      return report;
    }());
  REQUIRE((rmse.array() < RMSE_EXPECTATION.array()).all());
}

TEST_CASE("Rubric filtered measruemenets", "[require]") {
  std::vector<MeasurementFilter> filters{
    MeasurementFilter().add(Radar::Tag).add(Laser::Tag),
    MeasurementFilter().add(Radar::Tag),
    MeasurementFilter().add(Laser::Tag)
  };
  std::vector<Truth::x> rmses;
  std::transform(begin(filters), end(filters),
                 std::back_inserter(rmses),
                 [](const MeasurementFilter& mf) {
                   rmse = Truth::x();
                   std::ifstream f(input.c_str());
                   Application<FileProtocol> app(mf);
                   for(std::string s; std::getline(f, s); ) {
                     app.ProcessMessage(s);
                   }
                   return rmse;
                 });
  INFO([&rmses](){
      Eigen::MatrixXd r(rmses[0].size(), rmses.size());
      for(size_t i = 0; i < rmses.size(); ++i) r.col(i) = rmses[i];
      return r;
    }());
  REQUIRE((rmses[0].array() < rmses[1].array()).any());
  REQUIRE((rmses[0].array() < rmses[2].array()).any());
}

TEST_CASE("Rubric NIS", "[require]") {
  constexpr double radar_low = 0.35;
  constexpr double radar_high = 7.82;
  constexpr double laser_low = 0.10;
  constexpr double laser_high = 5.99;
  constexpr double th = 0.8;

  std::ifstream f(input.c_str());
  Application<FileProtocol> app(MeasurementFilter().add(Laser::Tag).add(Radar::Tag));
  for(std::string s; std::getline(f, s); ) {
    app.ProcessMessage(s);
  }

  double radar_nis = std::count_if(std::begin(nis[Radar::Tag]), std::end(nis[Radar::Tag]),
                                 [radar_low, radar_high](double v) {
                                   return v >= radar_low && v <= radar_high;
                                 });
  double laser_nis = std::count_if(std::begin(nis[Laser::Tag]), std::end(nis[Laser::Tag]),
                                 [laser_low, laser_high](double v) {
                                   return v >= laser_low && v <= laser_high;
                                 });

  REQUIRE(Approx(th) <= radar_nis/nis[Radar::Tag].size());
  REQUIRE(Approx(th) <= laser_nis/nis[Radar::Tag].size());  
}
