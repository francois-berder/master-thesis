#include "LinearEigenTriangulation.hpp"
#include "Utils.hpp"

using namespace Eigen;

std::vector<Vector3d> LinearEigenTriangulation::triangulate(Eigen::Matrix3d &rotation, Eigen::Vector3d &translation, std::vector<std::pair<Vector2d, Vector2d>> &points)
{
    Matrix<double, 3, 4> leftProjectionMatrix = computeLeftProjectionMatrix();
    Matrix<double, 3, 4> rightProjectionMatrix = computeRightProjectionMatrix(rotation, translation);

    std::vector<Vector3d> points3D;
    for(auto &p : points)
    {
        Vector2d &lp = p.first;
        Vector2d &rp = p.second;

        Matrix4d A;
        A.row(0) = leftProjectionMatrix.row(2).transpose() * lp(0) - leftProjectionMatrix.row(0).transpose();
        A.row(1) = leftProjectionMatrix.row(2).transpose() * lp(1) - leftProjectionMatrix.row(1).transpose();
        A.row(2) = rightProjectionMatrix.row(2).transpose() * rp(0) - rightProjectionMatrix.row(0).transpose();
        A.row(3) = rightProjectionMatrix.row(2).transpose() * rp(1) - rightProjectionMatrix.row(1).transpose();

        JacobiSVD<Matrix4d> svd(A, ComputeFullV);
        Vector3d point = svd.matrixV().col(3).block(0,0,3,1) / svd.matrixV()(3,3);
        points3D.push_back(point);
    }

    return points3D;
}
