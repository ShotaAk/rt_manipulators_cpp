// Copyright 2022 RT Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// For getch(), kbhit()
// Ref: https://github.com/ROBOTIS-GIT/DynamixelSDK/blob/master/c%2B%2B/example/protocol2.0/read_write/read_write.cpp
#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#define ESC_ASCII_VALUE 0x1b

#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "rt_manipulators_cpp/hardware.hpp"
#include "rt_manipulators_cpp/kinematics.hpp"
#include "rt_manipulators_cpp/kinematics_utils.hpp"
#include "rt_manipulators_cpp/link.hpp"
#include "rt_manipulators_dynamics.hpp"

int getch() {
  // Ref: https://github.com/ROBOTIS-GIT/DynamixelSDK/blob/c7e1eb71c911b87f7bdeda3c2c9e92276c2b4627/c%2B%2B/example/protocol2.0/read_write/read_write.cpp#L100-L114
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
}

int kbhit(void) {
  // Ref: https://github.com/ROBOTIS-GIT/DynamixelSDK/blob/c7e1eb71c911b87f7bdeda3c2c9e92276c2b4627/c%2B%2B/example/protocol2.0/read_write/read_write.cpp#L116-L143
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if (ch != EOF) {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
}

void set_arm_joint_positions(std::vector<manipulators_link::Link> & links,
                             std::vector<double> positions) {
  // リンクにarmジョイントの現在角度をセットする
  if (positions.size() != 6) {
    std::cerr << "引数positionsには6個のジョイント角 度をセットしてください" << std::endl;
    return;
  }

  int start_id = 2;  // Link1
  for (int i=0; i < positions.size(); i++) {
    links[start_id + i].q = positions[i];
  }
}

kinematics_utils::q_list_t impedance(
  kinematics_utils::links_t & links,
  const Eigen::Vector3d & target_pos, const Eigen::Matrix3d & target_R,
  const Eigen::Vector3d & target_vel, const Eigen::Vector3d & target_omega,
  const Eigen::MatrixXd & D, const Eigen::MatrixXd & K) {

  kinematics_utils::q_list_t tau_list = {
    {2, 0.0},
    {3, 0.0},
    {4, 0.0},
    {5, 0.0},
    {6, 0.0},
    {7, 0.0},
  };

  const int EEF_LINK_ID = 7;

  // 情報の更新
  auto present_pos = links[EEF_LINK_ID].p;
  auto present_R = links[EEF_LINK_ID].R;
  static Eigen::Vector3d prev_pos = present_pos;
  static Eigen::Matrix3d prev_R = present_R;

  auto present_vel = present_pos - prev_pos;
  auto present_omega = kinematics_utils::calc_error_R(present_R, prev_R);

  prev_pos = present_pos;
  prev_R = present_R;

  Eigen::VectorXd diff_r(6);
  diff_r << present_pos - target_pos,
    kinematics_utils::calc_error_R(present_R, target_R);

  Eigen::VectorXd diff_dr(6);
  diff_dr << present_vel - target_vel,
    present_omega - target_omega;

  auto J = kinematics_utils::calc_basic_jacobian(links, EEF_LINK_ID);
  auto tau = -J.transpose() * (D * diff_dr + K * diff_r);

  // std::cout << "vel X: " <<  diff_dr[0] << "\t[m/s]" << std::endl;
  // std::cout << "vel Y: " <<  diff_dr[1] << "\t[m/s]" << std::endl;
  // std::cout << "vel Z: " <<  diff_dr[2] << "\t[m/s]" << std::endl;
  // std::cout << "姿勢 Y:" << present_omega[1] * 180.0 / M_PI << "\t[deg]" << std::endl;
  // std::cout << "姿勢 X:" << present_omega[2] * 180.0 / M_PI << "\t[deg]" << std::endl;

  // ここひどいコード
  for (int i = 0; i < 6; i++) {
    tau_list[i+2] = tau[i];
  }

  return tau_list;
}

int main() {
  std::cout << "現在姿勢をもとに重力補償トルクを計算し、"
            << "CRANE-X7のサーボモータに入力するサンプルです" << std::endl;

  std::string port_name = "/dev/ttyUSB0";
  int baudrate = 4000000;
  std::string hardware_config_file = "../config/crane-x7_current_6dof.yaml";
  std::string link_config_file = "../config/crane-x7_links_6dof.csv";

  auto links = kinematics_utils::parse_link_config_file(link_config_file);
  kinematics::forward_kinematics(links, 1);

  rt_manipulators_cpp::Hardware hardware(port_name);
  if (!hardware.connect(baudrate)) {
    std::cerr << "ロボットとの接続に失敗しました." << std::endl;
    return -1;
  }

  if (!hardware.load_config_file(hardware_config_file)) {
    std::cerr << "コンフィグファイルの読み込みに失敗しました." << std::endl;
    return -1;
  }

  // 関節可動範囲の設定
  for (auto link_id : kinematics_utils::find_route(links, 8)) {
    hardware.get_max_position_limit(links[link_id].dxl_id, links[link_id].max_q);
    hardware.get_min_position_limit(links[link_id].dxl_id, links[link_id].min_q);
  }

  if (!hardware.torque_on("arm")) {
    std::cerr << "armグループのトルクをONできませんでした." << std::endl;
    return -1;
  }

  if (!hardware.torque_on("sub_joints")) {
    std::cerr << "sub_jointsグループのトルクをONできませんでした." << std::endl;
    return -1;
  }

  std::cout << "read/writeスレッドを起動します." << std::endl;
  std::vector<std::string> group_names = {"arm", "sub_joints"};
  if (!hardware.start_thread(group_names, std::chrono::milliseconds(10))) {
    std::cerr << "スレッドの起動に失敗しました." << std::endl;
    return -1;
  }

  std::cout << "5秒後に重力補償トルクをサーボモータへ入力します" << std::endl;
  std::cout << "終了する場合はEscキーを押してください" << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(5));

  // 使用しない軸の目標角度を0 degにする
  hardware.set_position("joint3", 0);

  kinematics_utils::q_list_t tau_g_list;
  const kinematics_utils::link_id_t EEF_BASE_LINK_ID = 7;
  // トルク・電流比 A/Nm
  // Dynamixelのe-manualに記載されたパラメータをもとに微調整しています
  samples03_dynamics::torque_to_current_t torque_to_current = {
    {2, 1.0 / 2.20},
    {3, 1.0 / 3.60},
    {4, 1.0 / 2.20},
    {5, 1.0 / 2.20},
    {6, 1.0 / 2.20},
    {7, 1.0 / 2.20},
    {8, 1.0 / 2.20}
  };

  while (1) {
    if (kbhit()) {
      if (getch() == ESC_ASCII_VALUE) {
        std::cout << "Escが入力されました" << std::endl;
        break;
      }
    }

    // 姿勢を更新
    std::vector<double> positions;
    if (hardware.get_positions("arm", positions)) {
      set_arm_joint_positions(links, positions);
      kinematics::forward_kinematics(links, 1);
    }

    // ここで重力補償分の電流値を計算
    samples03_dynamics::gravity_compensation(
      links, EEF_BASE_LINK_ID, tau_g_list);

    // インピーダンスを計算
    Eigen::VectorXd D(6);
    Eigen::VectorXd K(6);
    D <<
      3.0, 3.0, 3.0,
      0.5, 0.5, 0.5;
    K <<
      10.0, 10.0, 10.0,  // POS
      0.5, 0.5, 0.5;  // R
    Eigen::Vector3d target_pos(0.2, 0.0, 0.2);
    Eigen::Matrix3d target_R = kinematics_utils::rotation_from_euler_ZYX(0.0, M_PI, 0.0);
    Eigen::Vector3d target_vel(0.0, 0.0, 0.0);
    Eigen::Vector3d target_omega(0.0, 0.0, 0.0);
    auto tau_i_list = impedance(
      links,
      target_pos, target_R,
      target_vel, target_omega,
      D.asDiagonal(), K.asDiagonal());

    for (const auto & [target_id, tau_g] : tau_g_list) {
      // トルクを加算
      double tau = tau_g + tau_i_list[target_id];
      // トルクを電流値に変換
      auto dxl_id = links[target_id].dxl_id;
      auto q = torque_to_current.at(dxl_id) * tau;
      // 目標電流値を書き込む
      hardware.set_current(dxl_id, q);
    }
  }


  std::cout << "終了します" << std::endl;

  std::cout << "スレッドを停止します." << std::endl;
  hardware.stop_thread();

  if (!hardware.torque_off("arm")) {
    std::cerr << "armグループのトルクをOFFできませんでした." << std::endl;
  }

  std::cout << "CRANE-X7との接続を解除します." << std::endl;
  hardware.disconnect();
  return 0;
}
