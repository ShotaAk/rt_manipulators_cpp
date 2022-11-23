
#include "analytical_kinematics.hpp"

namespace analytical_kinematics {

void fk(const kinematics_utils::links_t & links, Eigen::Vector3d & eef_pos,
        Eigen::Matrix3d & eef_R) {
  // 同次変換行列をもとに順運動学を解き、link6原点位置とlink7原点姿勢を求める

  const double C1 = std::cos(links[2].q);
  const double C2 = std::cos(links[3].q);
  const double C3 = std::cos(links[4].q);
  const double C4 = std::cos(links[5].q);
  const double C5 = std::cos(links[6].q);
  const double C6 = std::cos(links[7].q);
  const double C7 = std::cos(links[8].q);

  const double S1 = std::sin(links[2].q);
  const double S2 = std::sin(links[3].q);
  const double S3 = std::sin(links[4].q);
  const double S4 = std::sin(links[5].q);
  const double S5 = std::sin(links[6].q);
  const double S6 = std::sin(links[7].q);
  const double S7 = std::sin(links[8].q);

  const double L12 = links[2].b.z() + links[3].b.z();
  const double L34 = links[4].b.z() + links[5].b.z();
  const double L56 = links[6].b.z() + links[7].b.z();

  // 書いてて気づいたけど、位置を決めるのは関節1 ~ 4だけだね
  eef_pos[0] = -C1*S2*L34 - C1*S2*C4*L56 - C1*C2*C3*S4*L56 + S1*S3*S4*L56;
  eef_pos[1] = -S1*S2*L34 - S1*S2*C4*L56 - S1*C2*C3*S4*L56 - C1*S3*S4*L56;
  eef_pos[2] = L12 + C2*L34 + C2*C4*L56 - S2*C3*S4*L56;

  eef_R(0, 0) = C1*C2*C3*C4*C5*C6*C7
    - S1*S3*C4*C5*C6*C7 - C1*S2*S4*C5*C6*C7 - C1*C2*S3*S5*C6*C7 - C1*C2*C3*S4*S6*C7 - C1*C2*C3*C4*S5*S7
    - S1*C3*S5*C6*C7 - C1*S2*C4*S6*C7 + S1*S3*S4*S6*C7 - C1*C2*S3*C5*S7 + S1*S3*C4*S5*S7 + C1*S2*S4*S5*S7
    - S1*C3*C5*S7;
  eef_R(1, 0) = S1*C2*C3*C4*C5*C6*C7
    + C1*S3*C4*C5*C6*C7 - S1*S2*S4*C5*C6*C7 - S1*C2*S3*S5*C6*C7 - S1*C2*C3*S4*S6*C7 - S1*C2*C3*C4*S5*S7
    + C1*C3*S5*C6*C7 - S1*S2*C4*S6*C7 - C1*S3*S4*S6*C7 - S1*C2*S3*C5*S7 - C1*S3*C4*S5*S7 + S1*S2*S4*S5*S7
    + C1*C3*C5*S7;
  eef_R(2, 0) = S2*C3*C4*C5*C6*C7
    + C2*S4*C5*C6*C7 - S2*S3*S5*C6*C7 - S2*C3*S4*S6*C7 - S2*C3*C4*S5*S7
    + C2*C4*S6*C7 - S2*S3*C5*S7 - C2*S4*S5*S7;

  eef_R(0, 1) = - C1*C2*C3*C4*C5*C6*S7
    + S1*S3*C4*C5*C6*S7 + C1*S2*S4*C5*C6*S7 + C1*C2*S3*S5*C6*S7 + C1*C2*C3*S4*S6*S7 - C1*C2*C3*C4*S5*C7
    + S1*C3*S5*C6*S7 + C1*S2*C4*S6*S7 - S1*S3*S4*S6*S7 - C1*C2*S3*C5*C7 + S1*S3*C4*S5*C7 + C1*S2*S4*S5*C7
    - S1*C3*C5*C7;
  eef_R(1, 1) = - S1*C2*C3*C4*C5*C6*S7
    - C1*S3*C4*C5*C6*S7 + S1*S2*S4*C5*C6*S7 + S1*C2*S3*S5*C6*S7 + S1*C2*C3*S4*S6*S7 - S1*C2*C3*C4*S5*C7
    - C1*C3*S5*C6*S7 + S1*S2*C4*S6*S7 + C1*S3*S4*S6*S7 - S1*C2*S3*C5*C7 - C1*S3*C4*S5*C7 + S1*S2*S4*S5*C7
    + C1*C3*C5*C7;
  eef_R(2, 1) = - S2*C3*C4*C5*C6*S7
    - C2*S4*C5*C6*S7 + S2*S3*S5*C6*S7 + S2*C3*S4*S6*S7 - S2*C3*C4*S5*C7
    - C2*C4*S6*S7 - S2*S3*C5*C7 - C2*S4*S5*C7;

  eef_R(0, 2) = - C1*C2*C3*C4*C5*S6
    - C1*C2*C4*S4*C6 + S1*S3*C4*C5*S6 + C1*S2*S4*C5*S6 + C1*C2*S3*S5*S6
    - C1*S2*C4*C6 + S1*S3*S4*C6 + S1*C3*S5*S6;
  eef_R(1, 2) = - S1*C2*C3*C4*C5*S6
    - S1*C2*C4*S4*C6 - C1*S3*C4*C5*S6 + S1*S2*S4*C5*S6 + S1*C2*S3*S5*S6
    - S1*S2*C4*C6 - C1*S3*S4*C6 - C1*C3*S5*S6;
  eef_R(2, 2) = - S2*C3*C4*C5*S6
    - S2*C3*S4*C6 - C2*S4*C5*S6 + S2*S3*S5*S6
    + C2*C4*C6;
}

}  // namespace analytical_kinematics
