#include "catch.hpp"
#include "../src/model.h"

namespace {
template<typename T, typename I>
void test_matrix_io(I init) {
  T v = init();
  REQUIRE(v.size() == 4);
  
  std::istringstream iss("1\t2\t3\t4");
  iss >> v;
  REQUIRE(v == (init() << 1, 2, 3, 4).finished());
}
}
  
TEST_CASE("Matrix IO", "[io]") {
  test_matrix_io<Eigen::Vector4d>([]{ return Eigen::Vector4d(); });
  test_matrix_io<Eigen::Matrix2d>([]{ return Eigen::Matrix2d(); });
  test_matrix_io<Eigen::VectorXd>([]{ return Eigen::VectorXd(4); });
  test_matrix_io<Eigen::MatrixXd>([]{ return Eigen::MatrixXd(2, 2); });
}

TEST_CASE("Model", "[math]") {
  Model::Interval i(0.2);

  REQUIRE(Model::F(i) == (Model::P() << 1, 0, i.count(), 0,
                                        0, 1, 0, i.count(),
                                        0, 0, 1, 0,
                                        0, 0, 0, 1).finished());
  REQUIRE(Model::G(i) == (Eigen::Matrix<double, Model::N, 2>()
                          << i.count() * i.count() / 2, 0,
                             0, i.count() * i.count() / 2,
                             i.count(), 0,
                             0, i.count()).finished());
}
