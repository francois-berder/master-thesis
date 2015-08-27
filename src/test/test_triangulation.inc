#include "Helper.hpp"
#include "LinearEigenTriangulation.hpp"
#include "MidPointTriangulation.hpp"
#include "OptimalTriangulation.hpp"

using namespace Eigen;
using namespace opengv;

namespace
{

void testTriangulationAlgorithm(Triangulation &algorithm)
{
    Matrix3d cameraMatrix = Helper::getCameraMatrix();
    std::vector<std::pair<Vector2d, Vector2d>> points = Helper::getPointsCorrespondencesTsukuba();

    transformation_t transformation = Helper::estimateTransformation(points, cameraMatrix);
    Matrix3d rotation = transformation.block(0,0,3,3);
    Vector3d translation = transformation.col(3);

    std::vector<std::pair<Vector2d, Vector2d>> pts;
    for(auto& e : points)
    {
        Vector3d xl(e.first(0), e.first(1), 1.);
        Vector3d xr(e.second(0), e.second(1), 1.);
        xl = cameraMatrix.inverse() * xl;
        xr = cameraMatrix.inverse() * xr;

        Vector2d lp(xl(0), xl(1));
        Vector2d rp(xr(0), xr(1));
        pts.push_back(std::make_pair(lp, rp));
    }

    std::vector<Vector3d> points3D = algorithm.triangulate(rotation, translation, pts);

    SECTION("nb of 3d points must equal nb points correspondences")
    {
        REQUIRE(points3D.size() == points.size());
    }

    SECTION("reprojection error must be small")
    {
        Matrix<double, 3, 4> leftProjectionMatrix = computeLeftProjectionMatrix();
        Matrix<double, 3, 4> rightProjectionMatrix = computeRightProjectionMatrix(rotation, translation);

        for(unsigned int i = 0; i < points3D.size(); ++i)
        {
            Vector3d &p = points3D[i];
            Vector4d X(p(0), p(1), p(2), 1.);
            Vector3d xl = cameraMatrix * leftProjectionMatrix * X;
            Vector3d xr = cameraMatrix * rightProjectionMatrix * X;
            xl /= xl(2);
            xr /= xr(2);
            Vector2d lp(xl(0), xl(1));
            Vector2d rp(xr(0), xr(1));

            double error = ((lp - points[i].first).norm() + (rp - points[i].second).norm()) / 2.;

            REQUIRE(error < 5.);
        }
    }
}

}

TEST_CASE("Linear Eigen triangulation", "[triangulate]")
{
    LinearEigenTriangulation let;
    testTriangulationAlgorithm(let);
}

TEST_CASE("Mid-Point triangulation", "[triangulate]")
{
    MidPointTriangulation mpt;
    testTriangulationAlgorithm(mpt);
}

TEST_CASE("Optimal triangulation", "[triangulate]")
{
    Matrix3d cameraMatrix = Helper::getCameraMatrix();
    OptimalTriangulation ot(cameraMatrix, cameraMatrix);
    testTriangulationAlgorithm(ot);
}

