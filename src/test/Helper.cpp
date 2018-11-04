#include <cmath>
#include <random>
#include "Helper.hpp"
#include "Utils.hpp"

using namespace Eigen;
using namespace opengv;
using namespace cv;

Vector3d Helper::createRandomTranslation()
{
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(0.0,1.0);

    Vector3d t(distribution(generator), distribution(generator), distribution(generator));
    t /= t.norm();

    return t;
}

Matrix3d Helper::createRandomRotation()
{
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(0.0,1.0);
    double angleX = distribution(generator);
    double angleY = distribution(generator);
    double angleZ = distribution(generator);

    Matrix3d r;
    r = AngleAxisd(angleX*M_PI, Vector3d::UnitX())
      * AngleAxisd(angleY*M_PI,  Vector3d::UnitY())
      * AngleAxisd(angleZ*M_PI, Vector3d::UnitZ());

    return r;
}

std::vector<Vector2d> Helper::getRandomPoints(const unsigned int N, double minX, double minY, double maxX, double maxY)
{
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distributionX(minX, maxX);
    std::uniform_real_distribution<double> distributionY(minY, maxY);

    std::vector<Vector2d> points(N);
    for(auto& p : points)
        p = Vector2d(distributionX(generator), distributionY(generator));

    return points;
}

Matrix3d Helper::getCameraMatrix()
{
    Matrix3d cameraMatrix;
    cameraMatrix << 768, 0., 319.5,
                    0., 768., 239.5,
                    0., 0., 1.;

    return cameraMatrix;
}

std::vector<std::pair<Vector2d, Vector2d>> Helper::getPointsCorrespondencesTsukuba()
{
    Mat leftFrame = imread("data/tsukuba/left.png");
    Mat rightFrame = imread("data/tsukuba/right.png");

    return findPoints(leftFrame, rightFrame, 500, 25.f);
}

transformation_t Helper::estimateTransformation(std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> &points, Matrix3d &cameraMatrix)
{
    bearingVectors_t xls, xrs;
    for(unsigned int i = 0; i < points.size(); ++i)
    {
        Vector2d &leftPoint = points[i].first;
        Vector2d &rightPoint = points[i].second;

        Vector3d xl(leftPoint(0), leftPoint(1), 1.);
        Vector3d xr(rightPoint(0), rightPoint(1), 1.);
        
        xl = cameraMatrix.inverse() * xl;
        xr = cameraMatrix.inverse() * xr;
        xl /= xl.norm();
        xr /= xr.norm();

        xls.push_back(xl);
        xrs.push_back(xr);
    }

    relative_pose::CentralRelativeAdapter adapter(xls, xrs);

    std::shared_ptr<sac_problems::relative_pose::CentralRelativePoseSacProblem>
        relposeproblem_ptr(
        new sac_problems::relative_pose::CentralRelativePoseSacProblem(
        adapter,
        sac_problems::relative_pose::CentralRelativePoseSacProblem::STEWENIUS));

    sac::Ransac<sac_problems::relative_pose::CentralRelativePoseSacProblem> ransac;
    ransac.sac_model_ = relposeproblem_ptr;
    ransac.threshold_ = (1.0 - cos(atan(std::sqrt(.5)*0.5/cameraMatrix(0,0))));
    ransac.max_iterations_ = 100000;
    ransac.computeModel();

    return ransac.model_coefficients_;
}

