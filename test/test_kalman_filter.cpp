#include "catch.hpp"

#include "../src/kalman_filter.h"
#include "../src/model.h"
#include "../src/radar.h"

TEST_CASE("Kalman Filter", "[math]") {
  /*
  Model::x x;
  x <<   5.7441,
         1.3800,
         2.2049,
         0.5015,
         0.3528;
  
  Model::P P;
  P <<     0.0043,   -0.0013,    0.0030,   -0.0022,   -0.0020,
          -0.0013,    0.0077,    0.0011,    0.0071,    0.0060,
           0.0030,    0.0011,    0.0054,    0.0007,    0.0008,
          -0.0022,    0.0071,    0.0007,    0.0098,    0.0100,
          -0.0020,    0.0060,    0.0008,    0.0100,    0.0123;

//  std::chrono::microseconds init{0};
  Model::State st{0, x, P};

  Model::x ex;
  ex <<   5.92276,
          1.41823,
          2.15593,
          0.489274,
          0.321338;

  Model::P eP;
  eP <<   0.00361579, -0.000357881, 0.00208316, -0.000937196, -0.00071727,
        -0.000357881,   0.00539867, 0.00156846,   0.00455342,  0.00358885,
          0.00208316,   0.00156846, 0.00410651,   0.00160333,  0.00171811,
        -0.000937196,   0.00455342, 0.00160333,   0.00652634,  0.00669436,
         -0.00071719,   0.00358884, 0.00171811,   0.00669426,  0.00881797;

  Radar::Package z{100000,
      (Radar::Measurement() << 
       5.9214,   //rho in m
       0.2187,   //phi in rad
       2.0062   //rho_dot in m/s
        ).finished()
      };

  auto r = kalman_filter::update<Model, Radar>(z, {true, st});
  const auto& rx = std::get<1>(std::get<1>(r));
  const auto& rP = std::get<2>(std::get<1>(r));
  
  auto dx = rx - ex;
  auto dP = rP - eP;
  INFO(rx);
  INFO(rP);
  REQUIRE((dx.array().abs() < 1e-5).all());
  REQUIRE((dP.array().abs() < 1e-5).all());
  */
}
