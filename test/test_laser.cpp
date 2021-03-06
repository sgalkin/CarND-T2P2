#include "catch.hpp"
#include "../src/model.h"
#include "../src/laser.h"

TEST_CASE("Laser computations", "[math]") {
  SECTION("H(X)") {
    constexpr size_t cols = 2*Model::N_A + 1;
    Eigen::Matrix<double, Model::N, cols> X;
    X <<
         5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,   5.9374,  5.9359,   5.93744,
           1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,     1.48,  1.4851,     1.486,
          2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,    2.204,  2.1702,    2.2049,
         0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367,  0.535048,
          0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214,  0.28687,   0.352,  0.318159;

    Eigen::Matrix<double, Laser::Size, cols> ez;
    ez <<
         5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,   5.9374,  5.9359,   5.93744,
           1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,     1.48,  1.4851,     1.486;

    auto z = Laser::H(X);
    auto delta = ez - z;
    INFO(delta);
    REQUIRE((delta.array().abs() < 1e-5).all());
  }
  SECTION("Cartesian") {
    Laser::Measurement z{Laser::Measurement::Random()};
    REQUIRE(z == Laser::Cartesian(z));
  }
}
