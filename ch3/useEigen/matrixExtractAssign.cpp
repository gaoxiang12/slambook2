# include <iostream>
# include <Eigen/Core>

#include <random> // 使用 C++11 随机数库




using namespace std;
using namespace Eigen;

#define MATRIX_SIZE 5

int main(int argc, char **argv) {
    // // 设置随机种子， time（0）作为随机种子不安全
    // srand(static_cast<unsigned int>(time(0)));

    std::random_device rd;                          // 获取一个高质量的随机种子
    std::default_random_engine generator(rd());     // 初始化随机数生成器
    std::uniform_real_distribution<double> distribution(-1.0, 1.0); // 均匀分布 [-1, 1]


    cout.precision(3);

    // 初始化 Eigen 矩阵
    MatrixXd bigMatrix(MATRIX_SIZE, MATRIX_SIZE); // 生成一个 rows x cols 的矩阵

    // 3. 使用随机数填充矩阵
    for (int i = 0; i < MATRIX_SIZE; ++i) {
        for (int j = 0; j < MATRIX_SIZE; ++j) {
            bigMatrix(i, j) = distribution(generator); // 填充每个元素
        }
    }

    cout << "The big matrix: \n" << bigMatrix << endl;

    Matrix3d extractedBlock = bigMatrix.block<3,3>(0,0);

    cout << "The extracted matrix block: \n" << extractedBlock << endl;

    extractedBlock.setIdentity();

    cout << "The assigned matrix block: \n" << extractedBlock << endl;

    return 0;
}
