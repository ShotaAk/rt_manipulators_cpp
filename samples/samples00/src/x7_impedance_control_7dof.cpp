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
  if (positions.size() != 7) {
    std::cerr << "引数positionsには7個のジョイント角 度をセットしてください" << std::endl;
    return;
  }

  int start_id = 2;  // Link1
  for (int i=0; i < positions.size(); i++) {
    links[start_id + i].q = positions[i];
  }
}

kinematics_utils::q_list_t impedance(
  kinematics_utils::links_t & links,
  const Eigen::Vector3d & present_pos, const Eigen::Matrix3d & present_R,
  const Eigen::Vector3d & target_pos, const Eigen::Matrix3d & target_R,
  const Eigen::Vector3d & target_vel, const Eigen::Vector3d & target_omega,
  const Eigen::MatrixXd & D, const Eigen::MatrixXd & K, const double max_tau) {

  kinematics_utils::q_list_t tau_list = {
    {2, 0.0},
    {3, 0.0},
    {4, 0.0},
    {5, 0.0},
    {6, 0.0},
    {7, 0.0},
    {8, 0.0},
  };
  static kinematics_utils::q_list_t prev_tau_list = tau_list;

  const int EEF_LINK_ID = 8;

  // 情報の更新
  static Eigen::Vector3d prev_pos = present_pos;
  static Eigen::Matrix3d prev_R = present_R;

  // リンク情報が更新されていなかったら、前回のtau_listを返す
  static std::chrono::system_clock::time_point prev_time = std::chrono::system_clock::now();
  if (present_pos.isApprox(prev_pos)) {
    return prev_tau_list;
  }

  // 経過時間を計測
  auto now_time = std::chrono::system_clock::now();
  auto elapsed_time = static_cast<double>(
    std::chrono::duration_cast<std::chrono::microseconds>(now_time - prev_time).count()) / 1e6;
  prev_time = now_time;

  auto present_vel = (present_pos - prev_pos) / elapsed_time;
  auto present_omega = kinematics_utils::calc_error_R(present_R, prev_R) / elapsed_time;

  prev_pos = present_pos;
  prev_R = present_R;

  Eigen::VectorXd diff_r(6);
  diff_r << present_pos - target_pos,
    kinematics_utils::calc_error_R(present_R, target_R);

  Eigen::VectorXd diff_dr(6);
  diff_dr << present_vel - target_vel,
    present_omega - target_omega;

  // 力にリミットを設ける
  Eigen::VectorXd gain = (D * diff_dr + K * diff_r);
  for (auto i=0; i < gain.size(); i++) {
    gain(i) = std::clamp(gain[i], -max_tau, max_tau);
  }
  auto J = kinematics_utils::calc_basic_jacobian(links, EEF_LINK_ID);
  auto tau = -J.transpose() * gain;

  // ここひどいコード
  for (int i = 0; i < 7; i++) {
    tau_list[i+2] = tau[i];
  }

  prev_tau_list = tau_list;
  return tau_list;
}

int main() {
  std::cout << "7軸インピーダンス制御" << std::endl;

  std::string port_name = "/dev/ttyUSB0";
  int baudrate = 4000000;
  std::string hardware_config_file = "../config/crane-x7_current.yaml";
  std::string link_config_file = "../config/crane-x7_links.csv";

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

  std::cout << "read/writeスレッドを起動します." << std::endl;
  std::vector<std::string> group_names = {"arm", "hand"};
  if (!hardware.start_thread(group_names, std::chrono::milliseconds(5))) {
    std::cerr << "スレッドの起動に失敗しました." << std::endl;
    return -1;
  }

  std::cout << "5秒後に重力補償トルクをサーボモータへ入力します" << std::endl;
  std::cout << "終了する場合はEscキーを押してください" << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(5));

  kinematics_utils::q_list_t tau_g_list;
  const kinematics_utils::link_id_t EEF_BASE_LINK_ID = 8;
  // トルク・電流比 A/Nm
  // Dynamixelのe-manualに記載されたパラメータをもとに微調整しています
  samples03_dynamics::torque_to_current_t torque_to_current = {
    {2, 1.0 / 2.20},
    {3, 1.0 / 4.00},
    {4, 1.0 / 2.20},
    {5, 1.0 / 2.50},
    {6, 1.0 / 2.20},
    {7, 1.0 / 2.60},
    {8, 1.0 / 2.20}
  };

  Eigen::Vector3d target_pos(0.2, 0.2, 0.2);
  Eigen::Matrix3d target_R = kinematics_utils::rotation_from_euler_ZYX(0.0, M_PI_2, 0.0);
  Eigen::Vector3d target_vel(0.0, 0.0, 0.0);
  Eigen::Vector3d target_omega(0.0, 0.0, 0.0);

  Eigen::VectorXd default_D(6);
  Eigen::VectorXd default_K(6);
  default_D <<
    4.0, 4.0, 4.0,
    0.002, 0.002, 0.002;
  default_K <<
    100.0, 100.0, 100.0,  // POS
    0.8, 0.8, 0.8;  // R

  bool impedance_X = true;
  bool impedance_Y = true;
  bool impedance_Z = true;
  bool impedance_R = true;

  while (1) {
    if (kbhit()) {
      auto key_value = getch();
      if (key_value == ESC_ASCII_VALUE) {
        std::cout << "Escが入力されました" << std::endl;
        break;

      } else if (key_value == 'x') {
        impedance_X = !impedance_X;
        std::cout << "X軸のインピーダンス:" << impedance_X << std::endl;

      } else if (key_value == 'y') {
        impedance_Y = !impedance_Y;
        std::cout << "Y軸のインピーダンス:" << impedance_Y << std::endl;

      } else if (key_value == 'z') {
        impedance_Z = !impedance_Z;
        std::cout << "Z軸のインピーダンス:" << impedance_Z << std::endl;

      } else if (key_value == 'r') {
        impedance_R = !impedance_R;
        std::cout << "R軸のインピーダンス:" << impedance_R << std::endl;
      }
    }

    // 姿勢を更新
    std::vector<double> positions;
    if (hardware.get_positions("arm", positions)) {
      set_arm_joint_positions(links, positions);
      kinematics::forward_kinematics(links, 1);
    }

    // 現在手先姿勢を取得
    auto eef_pos = links[EEF_BASE_LINK_ID].p;
    auto eef_R = links[EEF_BASE_LINK_ID].R;
    const Eigen::Vector3d TIP_B(0, 0, 0.085);
    auto eef_tip_pos = links[EEF_BASE_LINK_ID].R * TIP_B + links[EEF_BASE_LINK_ID].p;

    // ここで重力補償分の電流値を計算
    samples03_dynamics::gravity_compensation(
      links, EEF_BASE_LINK_ID, tau_g_list);

    // 手先が開いてるときは、目標姿勢を現在姿勢に更新する
    double hand_angle;
    hardware.get_position("joint_hand", hand_angle);
    if (hand_angle > M_PI_4) {
      target_pos = eef_tip_pos;
      target_R = eef_R;
    }

    // X,Y,Z軸、姿勢のインピーダンスをON/OFFできる
    Eigen::VectorXd D(6);
    Eigen::VectorXd K(6);
    D.setZero();
    K.setZero();

    if (impedance_X) {
      D[0] = default_D[0];
      K[0] = default_K[0];
    }

    if (impedance_Y) {
      D[1] = default_D[1];
      K[1] = default_K[1];
    }

    if (impedance_Z) {
      D[2] = default_D[2];
      K[2] = default_K[2];
    }

    if (impedance_R) {
      // もっとシンプルな書き方ありそう
      for (auto i=3; i < 6; i++) {
        D[i] = default_D[i];
        K[i] = default_K[i];
      }
    }

    // インピーダンスを計算
    auto tau_i_list = impedance(
      links,
      eef_tip_pos, eef_R,
      target_pos, target_R,
      target_vel, target_omega,
      D.asDiagonal(), K.asDiagonal(), 4.0);

    for (const auto & [target_id, tau_g] : tau_g_list) {
      // トルクを加算
      double tau = 0.0;
      tau += tau_g;  // 重力補償項を加算
      tau += tau_i_list[target_id];  // インピーダンス制御項を加算
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
