#include "Helper.hpp"
#include "Alignment.hpp"
#include "Utils.hpp"

using namespace cv;
using namespace Eigen;
using namespace opengv;

TEST_CASE("Alignment", "[alignImages]")
{
    Matrix3d cameraMatrix = Helper::getCameraMatrix();
    std::vector<std::pair<Vector2d, Vector2d>> points = Helper::getPointsCorrespondencesTsukuba();

    transformation_t transformation = Helper::estimateTransformation(points, cameraMatrix);
    Matrix3d rotation = transformation.block(0,0,3,3);
    Vector3d translation = transformation.col(3);
    translation /= translation.norm();

    Mat leftFrame = imread("data/tsukuba/left.png");
    Mat rightFrame = imread("data/tsukuba/right.png");

    Alignment alignmentAlgo(cameraMatrix, cameraMatrix);
    Mat leftAligned, rightAligned;
    Matrix3d Hr;
    std::tie(leftAligned, rightAligned, std::ignore, Hr) = alignmentAlgo.alignImages(rotation, translation, leftFrame, rightFrame);

    SECTION("right epipole must be mapped to infinity (1,0,0)^T")
    {
        Vector3d rightEpipole = computeRightEpipole(cameraMatrix, rotation, translation);
        Vector3d tmp = Hr * rightEpipole;
        REQUIRE(std::fabs(tmp(0) - 1.) <= 1e-12);
        REQUIRE(tmp(1) <= 1e-12);
        REQUIRE(tmp(2) <= 1e-12);
    }
}
