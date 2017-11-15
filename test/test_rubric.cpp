#include <fstream>
#include <string>
#include <iostream>

#include "catch.hpp"
#include "../src/application.h"
#include "../src/laser.h"
#include "../src/radar.h"

namespace {
  const std::string input = "../data/obj_pose-laser-radar-synthetic-input.txt";

  Model::x rmse;

  struct FileProtocol {
    static bool checkHeader(const std::string&) { return true; }

    static std::string getPayload(std::string message) { return message; }
    static std::string getMeasurement(std::string payload) { return payload; }

    static std::string formatResponse() { return std::string(); }
    static std::string formatResponse(const Model::x&, const Model::x& RMSE) {
      rmse = RMSE;
      return std::string();
    }
  };

  Model::x RMSE_EXPECTATION = []{
    return (Model::x() << .09, .10, .40, .30).finished();
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
  std::vector<Model::x> rmses;
  std::transform(begin(filters), end(filters),
                 std::back_inserter(rmses),
                 [](const MeasurementFilter& mf) {
                   rmse = Model::x();
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
  REQUIRE((rmses[0].array() < rmses[1].array()).all());
  REQUIRE((rmses[0].array() < rmses[2].array()).all());
}

TEST_CASE("Rubric NIS", "[require]") {
/* 
   The NIS of radar measurements must be between 0.35 and 7.81 in at least 80% of 
   all radar update steps.
*/
  REQUIRE(false);
}
