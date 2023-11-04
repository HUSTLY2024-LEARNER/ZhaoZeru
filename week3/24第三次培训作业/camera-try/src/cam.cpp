#include <iostream>
#include <ctime>
#include <Eigen/Core>
#include <Eigen/Dense>
using namespace std;
using namespace Eigen;

#define MATRIX_SIZE 50

int main(void)
{
    Matrix<double,2,3> matrix23d; //两行三列矩阵
    matrix23d.setZero();
    matrix23d << 1,2,3,4,5,6;

    //cout << "matrix23d = \n" << matrix23d << endl;
    for(int i = 0;i<2;i++)
    {
        for(int j = 0; j < 3; j++)
        {
            cout << matrix23d(i,j) << " ";
        }
        cout<<endl;
    }

    Vector3d v3d;

    Matrix3d matrix33d = Matrix3d::Zero(); // 初始化为0
    matrix33d << 1,2,3,4,5,6,7,8,9;
    cout << "matrix33d\n" << matrix33d <<endl;

    //动态矩阵
    int rows = 3, cols = 3;
    Matrix<double,Dynamic,Dynamic> matrixX;
    matrixX.resize(rows,cols);
    matrixX.setZero();
    cout << "动态矩阵 = \n" << matrixX <<endl;
    rows = 3, cols = 4;
    matrixX.resize(rows,cols);
    matrixX.setZero();
    cout<< "动态矩阵 = \n" << matrixX <<endl;

    //随机数矩阵
    Matrix3d randomMatrix3d = Matrix3d::Zero();
    randomMatrix3d.setRandom();
    cout << "随机数矩阵 = \n" << randomMatrix3d <<endl; 

    //矩阵和向量乘法
    v3d << 3,2,1;
    cout<< "matrix23d * v3d = " << (matrix23d * v3d).transpose() <<endl;

    //不同类型的矩阵和向量进行运算
    //进行矩阵转换
    Matrix<float,2,3> matrix23f;
    matrix23f << 1,2,3,4,5,6;
    cout << "matrix23f * v3d" << (matrix23f.cast<double>() * v3d).transpose() <<endl;

     //转置
    cout << "matrix23d的转置 = \n" << matrix23d.transpose() << endl;
    //元素和
    cout << "matrix23d元素求和 = " << matrix23d.sum() << endl;
    //迹
    cout << "matrix33d元素的迹 = " << matrix33d.trace() << endl;
    //数乘
    cout << "10 * matrix33d = \n" << 10*matrix33d << endl;
    //行列式
    cout << "matrix33d的行列式 = " << matrix33d.determinant() << endl;
    //逆     
    cout << "matrix33d的逆 = \n" << matrix33d.inverse() << endl;
    
     //特征值（实对称矩阵一定可以对角化）  
  //对于3维全1矩阵，理论特征值是0、0、3，计算机求解答案是-1.28198e-16、2.22045e-16、3，前两个特征值幂为-16，约等于0
    matrix33d << 1,1,1,1,1,1,1,1,1;
    cout << matrix33d << endl;
    SelfAdjointEigenSolver<Matrix3d> eigenSolver(matrix33d);
    //eigenSolver.compute(matrix33d);
    cout << "特征值 = \n" << eigenSolver.eigenvalues().transpose() << endl;
    cout << "特征向量 = \n" << eigenSolver.eigenvectors() << endl;
        
     
    //解方程：matrixNN * x = vNd（待求x）
    //直接求逆运算量大，可利用矩阵分解求解（如方法2、3）
    Matrix<double, MATRIX_SIZE, MATRIX_SIZE> matrixNN = MatrixXd::Random(MATRIX_SIZE, MATRIX_SIZE);
    matrixNN = matrixNN * matrixNN.transpose();	//保证半正定
    Matrix<double, MATRIX_SIZE, 1> vNd = MatrixXd::Random(MATRIX_SIZE, 1);
    
    //法1：直接求逆（x = matrixNN^(-1) * vNd）
    clock_t timeStart = clock();
    Matrix<double, MATRIX_SIZE, 1> x = matrixNN.inverse() * vNd;
    cout << "解方程，直接求逆用时:" << (clock() - timeStart) / (double)CLOCKS_PER_SEC << "s" << endl;
    cout << "结果为:\n" << x.transpose() << endl;
    
    //法2：QR分解
    timeStart = clock();
    x = matrixNN.colPivHouseholderQr().solve(vNd);
    cout << "解方程，QR分解用时：" << (clock() - timeStart) / (double)CLOCKS_PER_SEC << "s" << endl;
    cout << "结果为:\n" << x.transpose() << endl;
    
    //法3：cholesky分解（用于正定阵）
    timeStart = clock();
    x = matrixNN.ldlt().solve(vNd);
    cout << "解方程，cholesky分解用时：" << (clock() - timeStart) / (double)CLOCKS_PER_SEC << "s" << endl;
    cout << "结果为:\n" << x.transpose() << endl;
    return 0; 

}