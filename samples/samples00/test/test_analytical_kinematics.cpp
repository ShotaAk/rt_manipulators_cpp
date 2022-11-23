

#include <random>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "rt_manipulators_cpp/kinematics.hpp"
#include "rt_manipulators_cpp/kinematics_utils.hpp"
#include "rt_manipulators_cpp/link.hpp"
#include "analytical_kinematics.hpp"


void expect_matrix_approximation(
  const Eigen::Matrix3d & actual, const Eigen::Matrix3d & expected,
  const std::string & message = "") {
  EXPECT_TRUE(actual.isApprox(expected))
    << message << std::endl
    << "actual:" << std::endl << actual << std::endl
    << "expected:" << std::endl << expected;
}

void expect_vector_approximation(
  const Eigen::Vector3d & actual, const Eigen::Vector3d & expected,
  const std::string & message = "") {
  EXPECT_TRUE(actual.isApprox(expected))
    << message << std::endl
    << "actual:" << std::endl << actual << std::endl
    << "expected:" << std::endl << expected;
}

void expect_FK(
  std::vector<manipulators_link::Link> & links,
  const int & target_link, const double & target_q,
  const Eigen::Vector3d & expected_p, const Eigen::Matrix3d & expected_R,
  const std::string & message = "", const int & expect_link = 8) {
  links[target_link].q = target_q;
  kinematics::forward_kinematics(links, 1);
  expect_vector_approximation(links[expect_link].p, expected_p, message);
  expect_matrix_approximation(links[expect_link].R, expected_R, message);
  links[target_link].q = 0.0;
}


class X7KinematicsFixture: public ::testing::Test {
 protected:
  virtual void SetUp() {
    links = kinematics_utils::parse_link_config_file("../../config/crane-x7_links.csv");

    // 関節の可動範囲を制限
    links[2].min_q = -150 * M_PI / 180.0;
    links[2].max_q = 150 * M_PI / 180.0;
    links[3].min_q = -90 * M_PI / 180.0;
    links[3].max_q = 90 * M_PI / 180.0;
    links[4].min_q = -150 * M_PI / 180.0;
    links[4].max_q = 150 * M_PI / 180.0;
    links[5].min_q = -160 * M_PI / 180.0;
    links[5].max_q = 0 * M_PI / 180.0;
    links[6].min_q = -150 * M_PI / 180.0;
    links[6].max_q = 150 * M_PI / 180.0;
    links[7].min_q = -90 * M_PI / 180.0;
    links[7].max_q = 90 * M_PI / 180.0;
    links[8].min_q = -160 * M_PI / 180.0;
    links[8].max_q = 160 * M_PI / 180.0;
  }

  virtual void TearDown() {
  }

  std::vector<manipulators_link::Link> links;
};



TEST_F(X7KinematicsFixture, fk) {
  // link6原点の位置・姿勢を検査する
  // ライブラリの順運動学関数の結果と一致することを期待する

  Eigen::Vector3d link6_pos(0.0, 0.0, 0.0);
  Eigen::Matrix3d link6_R;
  link6_R.setZero();

  // 初期姿勢でもFKが解けることを期待
  kinematics::forward_kinematics(links, 1);
  analytical_kinematics::fk(links, link6_pos, link6_R);
  expect_vector_approximation(link6_pos, links[7].p);

  // 乱数を使って適当に関節を動かしてもFKが解けることを期待
  std::random_device seed_gen;
  std::default_random_engine engine(seed_gen());
  std::uniform_real_distribution<> dist(-160*M_PI/180.0, 160*M_PI/180.0);

  for (auto steps=0; steps < 10; steps++) {
    for (auto i=2; i < 9; i++) {
      links[i].set_q_within_limit(dist(engine));
    }

    kinematics::forward_kinematics(links, 1);
    analytical_kinematics::fk(links, link6_pos, link6_R);
    expect_vector_approximation(link6_pos, links[7].p);
  }
}

