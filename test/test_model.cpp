#include "catch.hpp"
#include "../src/model.h"

namespace {
template<typename T, typename I>
void test_matrix_io(I init) {
  T v{init()};
  REQUIRE(v.size() == 4);
  
  std::istringstream iss{"1\t2\t3\t4"};
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

TEST_CASE("Model f(x, nu)", "[math]") {
  SECTION("PSI != 0") {
    constexpr Model::Interval dt{0.1};
    Eigen::Matrix<double, Model::N_A, 2*Model::N_A + 1> X;
    X <<
      5.7441,  5.85768,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.63052,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,
      1.38,  1.34566,  1.52806,     1.38,     1.38,     1.38,     1.38,     1.38,   1.41434,  1.23194,     1.38,     1.38,     1.38,     1.38,     1.38,
      2.2049,  2.28414,  2.24557,  2.29582,   2.2049,   2.2049,   2.2049,   2.2049,   2.12566,  2.16423,  2.11398,   2.2049,   2.2049,   2.2049,   2.2049,
      0.5015,  0.44339, 0.631886, 0.516923, 0.595227,   0.5015,   0.5015,   0.5015,   0.55961, 0.371114, 0.486077, 0.407773,   0.5015,   0.5015,   0.5015,
      0.3528, 0.299973, 0.462123, 0.376339,  0.48417, 0.418721,   0.3528,   0.3528,  0.405627, 0.243477, 0.329261,  0.22143, 0.286879,   0.3528,   0.3528,
      0,        0,        0,        0,        0,        0,  0.34641,        0,         0,        0,        0,        0,        0, -0.34641,        0,
      0,        0,        0,        0,        0,        0,        0,  0.34641,         0,        0,        0,        0,        0,        0, -0.34641;

    Eigen::Matrix<double, Model::N, 2*Model::N_A + 1> eR;
    eR <<
      5.93553,  6.06251,  5.92217,   5.9415,  5.92361 , 5.93516, 5.93705,  5.93553,  5.80832,  5.94481,  5.92935,  5.94553,  5.93589, 5.93401,  5.93553,
      1.48939,  1.44673,  1.66484,  1.49719,    1.508,  1.49001, 1.49022,  1.48939,   1.5308,  1.31287,  1.48182,  1.46967,  1.48876, 1.48855,  1.48939,
      2.2049,  2.28414,  2.24557,  2.29582,   2.2049,   2.2049, 2.23954,   2.2049,  2.12566,  2.16423,  2.11398,   2.2049,   2.2049, 2.17026,   2.2049,
      0.53678, 0.473387, 0.678098, 0.554557, 0.643644, 0.543372, 0.53678, 0.538512, 0.600173, 0.395462, 0.519003, 0.429916, 0.530188, 0.53678, 0.535048,
      0.3528, 0.299973, 0.462123, 0.376339,  0.48417, 0.418721,  0.3528, 0.387441, 0.405627, 0.243477, 0.329261,  0.22143, 0.286879,  0.3528, 0.318159;
  
    auto R = Model::F(X, dt);
    auto delta = R - eR;
    INFO("Delta:\n" << delta << "\nR:\n" << R);
    REQUIRE((delta.array().abs() < 1e-5).all());
  }
  SECTION("PSI == 0") {
    constexpr Model::Interval dt{1};
    Eigen::Matrix<double, Model::N_A, 3> X;
    X <<
      0, 0, 0,
      0, 0, 0,
      1, 1, 1,
      0, M_PI/2, M_PI/4,
      0, 0, 0,
      0, 0, 0,
      0, 0, 0;

    Eigen::Matrix<double, Model::N, 3> eR;
    eR <<
      1, 0, sqrt(2)/2,
      0, 1, sqrt(2)/2,
      1, 1, 1,
      0, M_PI/2, M_PI/4,
      0, 0, 0;

    auto R = Model::F(X, dt);
    auto delta = R - eR;
    INFO("Delta:\n" << delta << "\nR:\n" << R);
    REQUIRE((delta.array().abs() < 1e-15).all());
  }
}

TEST_CASE("Model augmentation", "[math]") {
  SECTION("State augmentation") {
    Model::x x{Model::x::Random()};
    Model::xa exa{(Model::xa() << x, 0, 0).finished()};
    REQUIRE(Model::Augment(x) == exa);
  }
  SECTION("Covariance augmentation") {
    Model::P P{Model::P::Random()};
    Model::Pa ePa{Model::Pa::Zero()};
    ePa.topLeftCorner<Model::N, Model::N>() = P;
    ePa.bottomRightCorner<Model::N_A - Model::N,
                          Model::N_A - Model::N>() = Model::Q;
    REQUIRE(Model::Augment(P) == ePa);
  }
}
