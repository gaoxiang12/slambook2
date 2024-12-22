# include <iostream>
# include <cmath>
# include <Eigen/Core>
# include <Eigen/Geometry>
# include "sophus/se3.hpp"

using namespace std;
using namespace Eigen;

/// 本程序演示sophus的基本用法
int main(int argc, char **argv) {
  // 沿Z轴转90度的旋转矩阵
  Matrix3d R = AngleAxisd(M_PI/2,Vector3d(0,0,1)).toRotationMatrix();
  // 或者四元数
  Quaterniond q(R);
  Sophus::SO3d SO3_R(R);  // Sophus::SO3d可以直接从旋转矩阵构造
  Sophus::SO3d SO3_q(q);  // 也可以通过四元数构造
  // 二者是等价的
  cout << "SO(3) from matrix: \n" << SO3_R.matrix() << endl;
  cout << "SO(3) from quaternion: \n" << SO3_q.matrix() << endl;
  cout << "they are equal" << endl;

  // 使用对数映射获得它的李代数
  Vector3d so3 = SO3_R.log();
  cout << "so3 = " << so3.transpose() << endl;
  // hat为向量到反对称矩阵
  cout << "so3 hat=\n" << Sophus::SO3d::hat(so3) << endl;
  // 相对的，vee为反对称到向量
  cout << "so3 hat vee= " << Sophus::SO3d::vee(Sophus::SO3d::hat(so3)).transpose() << endl;

  // 增量扰动模型的更新
  Vector3d update_so3(1e-4,0,0); //假设更新量为这么多


  return 0;
}