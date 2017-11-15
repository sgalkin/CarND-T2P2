#include "../src/measurement.h"

#include "../src/Eigen/Dense"
#include "catch.hpp"
#include <iostream>

TEST_CASE("Package IO", "[io]") {
  std::istringstream iss("1\t2\t3\t4\t1234567890");
  struct Fake : MeasurementS<4> {};
  Fake::Package p;
  iss >> p;
  REQUIRE(std::get<0>(p) == std::chrono::microseconds(1234567890));
  REQUIRE(std::get<1>(p) == (Fake::Measurement() << 1,2,3,4).finished());
}


