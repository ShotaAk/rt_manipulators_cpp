
#include "analytical_kinematics.hpp"

namespace analytical_kinematics {

void fk(const kinematics_utils::links_t & links, Eigen::Vector3d & eef_pos,
        Eigen::Matrix3d & eef_R) {
  // 同次変換行列をもとに順運動学を解き、link6原点の座標と姿勢を求める

  const double C1 = std::cos(links[2].q);
  const double C2 = std::cos(links[3].q);
  const double C3 = std::cos(links[4].q);
  const double C4 = std::cos(links[5].q);

  const double S1 = std::sin(links[2].q);
  const double S2 = std::sin(links[3].q);
  const double S3 = std::sin(links[4].q);
  const double S4 = std::sin(links[5].q);

  const double L12 = links[2].b.z() + links[3].b.z();
  const double L34 = links[4].b.z() + links[5].b.z();
  const double L56 = links[6].b.z() + links[7].b.z();

  // 書いてて気づいたけど、位置を決めるのは関節1 ~ 4だけだね
  eef_pos[0] = -C1*S2*L34 - C1*S2*C4*L56 - C1*C2*C3*S4*L56 + S1*S3*S4*L56;
  eef_pos[1] = -S1*S2*L34 - S1*S2*C4*L56 - S1*C2*C3*S4*L56 - C1*S3*S4*L56;
  eef_pos[2] = L12 + C2*L34 + C2*C4*L56 - S2*C3*S4*L56;
}

}  // namespace analytical_kinematics
