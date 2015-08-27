#include "MidPointTriangulation.hpp"

using namespace Eigen;

std::vector<Vector3d> MidPointTriangulation::triangulate(Matrix3d &rotation, Vector3d &translation, std::vector<std::pair<Vector2d, Vector2d>> &points)
{
    Matrix3d invM = rotation.inverse();
    Vector3d rightCameraPosition = translation;

    std::vector<Vector3d> points3D;
    for(auto& e : points)
    {
        Vector3d xl(e.first(0), e.first(1), 1.);
        Vector3d xr(e.second(0), e.second(1), 1.);

        Matrix<double, 3, 2> A;
        A.col(0) = xl;
        A.col(1) = -(invM * xr);

        JacobiSVD<Matrix<double, 3, 2>> svd(A, ComputeFullU | ComputeFullV);
        Vector2d tmp = svd.solve(translation);
        Vector3d point = (tmp(0) * xl + rightCameraPosition + tmp(1) * invM * xr) / 2.;
        points3D.push_back(point);
    }

    return points3D;
}
