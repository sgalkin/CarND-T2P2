#include "catch.hpp"
#include "../src/model.h"
#include "../src/radar.h"

TEST_CASE("Radar computations", "[math]") {
  SECTION("Origin Hp") {
    Model::x origin = (Model::x() << 0, 0, 1, 1).finished();
    REQUIRE_NOTHROW(Radar::Hp(origin));
  }

  SECTION("Origin Hj") {
    Model::x origin = (Model::x() << 0, 0, 1, 1).finished();
    REQUIRE_NOTHROW(Radar::Hj(origin));
  }

  SECTION("Normalize > M_PI") {
    Radar::Measurement z = (Radar::Measurement() << 1, 1.5*M_PI, 1).finished();
    REQUIRE(Radar::Normalize(z)(1) == Approx(-0.5*M_PI));
  }

  SECTION("Normalize < -M_PI") {
    Radar::Measurement z = (Radar::Measurement() << 1, -1.25*M_PI, 1).finished();
    REQUIRE(Radar::Normalize(z)(1) == Approx(0.75*M_PI));
  }

  SECTION("Cartesian Y") {
    Radar::Measurement z = (Radar::Measurement() << 1, 0, 1).finished();
    auto c = Radar::Cartesian(z);
    REQUIRE(c(0) == Approx(0));
    REQUIRE(c(1) == Approx(1));
  }

  SECTION("Cartesian -X") {
    Radar::Measurement z = (Radar::Measurement() << 1, -M_PI/2, 1).finished();
    auto c = Radar::Cartesian(z);

    REQUIRE(c(0) == Approx(-1));
    REQUIRE(c(1) == Approx(0).margin(0.0001));
  }

  SECTION("Cartesian XY") {
    Radar::Measurement z = (Radar::Measurement() << 8, M_PI/6, 1).finished();
    auto c = Radar::Cartesian(z);
    REQUIRE(c(0) == Approx(4).margin(0.0001));
    REQUIRE(c(1) == Approx(sqrt(48)).margin(0.0001));
  }
}
