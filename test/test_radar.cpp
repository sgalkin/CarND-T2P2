#include "catch.hpp"
#include "../src/model.h"
#include "../src/radar.h"

TEST_CASE("Radar computations", "[math]") {
  SECTION("H(X)") {
    constexpr size_t cols = 2*Model::N_A + 1;
    Eigen::Matrix<double, Model::N, cols> X;
    X <<
         5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,   5.9374,  5.9359,   5.93744,
           1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,     1.48,  1.4851,     1.486,
          2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,    2.204,  2.1702,    2.2049,
         0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367,  0.535048,
          0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214,  0.28687,   0.352,  0.318159;

    Eigen::Matrix<double, Radar::Size, cols> ez;
    ez <<
       6.11908,  6.23346,  6.15315,  6.12835,  6.11436,  6.11908,  6.12218,  6.11908,  6.00792,  6.08839,  6.11255,  6.12488,  6.11908,  6.11886,  6.12057,
      0.244289,  0.23371, 0.273165, 0.246166, 0.248461, 0.244289, 0.245307, 0.244289, 0.257001, 0.216927, 0.244336, 0.241934, 0.244289, 0.245157, 0.245239,
       2.11044,  2.21881,  2.06391,   2.1875,  2.03413,  2.10616,  2.14509,  2.10929,  2.00166,   2.1298,  2.03466,  2.16518,  2.11454,  2.07862,  2.11295;

    auto z = Radar::H(X);
    auto delta = ez - z;
    INFO(delta);
    REQUIRE((delta.array().abs() < 1e-5).all());
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
