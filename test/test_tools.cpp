#include "catch.hpp"
#include "../src/Eigen/Dense"
#include "../src/tools.h"

namespace {
template<typename T, typename I>
void test(I init) {
  using S2D = std::vector<T>;
  
  SECTION("Empty input throws") {
    REQUIRE_THROWS_AS(tools::RMSE(S2D(), S2D()), std::invalid_argument);
  }

  SECTION("Mistmatch throws") {
    S2D t { typename S2D::value_type() };
    REQUIRE_THROWS_AS(tools::RMSE(S2D(), t), std::invalid_argument);
  }

  SECTION("Simple evaluation") {
    S2D x { (init() << 0, 2, 0, 2).finished() };
    S2D y { (init() << 2, 0, 2, 0).finished() };
    auto r = tools::RMSE(x, y);
    REQUIRE(r == (init() << 2, 2, 2, 2).finished());
  }

  SECTION("Complex evaluation") {
    S2D x { 4, (init() << 0, 2, 0, 0).finished() };
    S2D y { 4, (init() << 0, 0, 0, 2).finished() };
    auto r = tools::RMSE(x, y);
    REQUIRE(r == (init() << 0, 2, 0, 2).finished());
  }
}
}
  
TEST_CASE("RMSE", "[math]") {
  test<Eigen::Vector4d>([]{return Eigen::Vector4d();});
  test<Eigen::VectorXd>([]{return Eigen::VectorXd(4);});
  test<Eigen::MatrixXd>([]{return Eigen::MatrixXd(2,2);});
}
