#include <iostream>
#include <eigen3/Eigen/Eigen>
using std::cout;
using std::endl;

// 试验8点法是否可行

int main()
{
    Eigen::Matrix<double, 16, 2> pts;
    pts << 345.594, 362.544,
            348.058, 362.533,
    361.331, 280.768,
            364.194, 279.328,
    300.677, 294.44,
            302.747, 292.371,
    507.7, 255.42,
            511.66, 253.409,
    308.549, 286.159,
            311.037, 283.673,
    492.673, 94.0541,
            494.254, 88.9834,
    74.4067, 192.188,
            74.4138, 190.784,
    221.099, 394.298,
            223.133, 394.311;
    std::vector<Eigen::Vector3d> p1s;
    std::vector<Eigen::Vector3d> p2s;

    for (int i = 0; i < 16; )
    {
        p1s.emplace_back(pts(i,0), pts(i,1), 1);
        p2s.emplace_back(pts(i+1,0), pts(i+1,1), 1);

        i += 2;
    }

    Eigen::Matrix<double, 8, 9> A;
    Eigen::Matrix<double, 1, 9> a = Eigen::Matrix<double, 1, 9>::Zero();
    for (int i = 0; i < 8; ++i)
    {
        double u1 = p1s[i].x(); double v1 = p1s[i].y();
        double u2 = p2s[i].x(); double v2 = p2s[i].y();
        a << u1*u2, u2*v1, u2, u1*v2, v1*v2, v2, u1, v1, 1;
        A.row(i) = a;
    }

//    cout << "A is\n" << A << endl;
    Eigen::Matrix3d F;
    Eigen::JacobiSVD<Eigen::Matrix<double, 8, 9>> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix<double, 9, 9> V = svd.matrixV();
    Eigen::Matrix<double, 8, 8> U = svd.matrixU();
    Eigen::Matrix<double, 8, 1> sigmas = svd.singularValues();
    Eigen::Matrix<double, 8, 9> S = Eigen::Matrix<double, 8, 9>::Zero();
    for (int i = 0; i < 8; ++i)
    {
        S(i,i) = sigmas[i];
    }

    Eigen::Matrix<double, 9, 1> h = V.col(8);
    cout << "A*h =\n" << (A*h).transpose() << endl;

    Eigen::Matrix3d iniF = Eigen::Matrix3d::Zero();
    F.row(0) = h.topRows(3).transpose();
    F.row(1) = h.block<3,1>(3,0).transpose();
    F.row(2) = h.bottomRows(3).transpose();
    cout << "h is\n" << h << endl;

    for (int i = 0; i < 8; ++i)
    {
        cout << p2s[i].transpose() * F * p1s[i] << " ";
    }

}