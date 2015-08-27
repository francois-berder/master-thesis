#include <random>
#include <cmath>
#include "Helper.hpp"
#include "Utils.hpp"

using namespace Eigen;

TEST_CASE("Utils", "[all]")
{
    SECTION("skewSym")
    {
        Vector3d v = Helper::createRandomTranslation();
        Matrix3d m = skewSym(v);

        REQUIRE(m(0,0) == 0.);
        REQUIRE(m(1,1) == 0.);
        REQUIRE(m(2,2) == 0.);

        REQUIRE(m(1,2) == -v(0));
        REQUIRE(m(2,1) == v(0));

        REQUIRE(m(0,2) == v(1));
        REQUIRE(m(2,0) == -v(1));

        REQUIRE(m(0,1) == -v(2));
        REQUIRE(m(1,0) == v(2));
    }

    SECTION("extractRotationTranslation")
    {
        Matrix3d cameraMatrix = Helper::getCameraMatrix();

        Matrix3d rotation = Helper::createRandomRotation();
        Vector3d translation = Helper::createRandomTranslation();

        Matrix3d F = cameraMatrix.inverse().transpose() * skewSym(translation) * rotation * cameraMatrix.inverse();

        std::vector<Vector2d> leftPoints = Helper::getRandomPoints(10, 0., 0., 640, 480);
        std::vector<std::pair<Vector2d, Vector2d>> points;
        for(auto& lp : leftPoints)
        {
            Vector3d xl(lp(0), lp(1), 1.);
            Vector3d l = F * xl;

            Vector3d xr(320., 240., 1.);
            xr(2) = -l(0)*320. - l(1)*240.;

            Vector2d rp(xr(0)/xr(2), xr(1)/xr(2));
            points.push_back(std::make_pair(lp, rp));
        }

        Matrix3d R;
        Vector3d T;
        std::tie(R, T) = extractRotationTranslation(F, cameraMatrix, cameraMatrix, points);

        REQUIRE((R-rotation).sum() < 1.);
        REQUIRE((T-translation).sum() < 0.1);
    }

    SECTION("findCardanAngles")
    {
        std::default_random_engine generator;
        std::uniform_real_distribution<double> distribution(-1.,1.);
        double expectedAngleX = distribution(generator) * M_PI;
        double expectedAngleY = distribution(generator) * M_PI / 2.;
        double expectedAngleZ = distribution(generator) * M_PI;

        Matrix3d rotation;
        rotation = AngleAxisd(expectedAngleZ, Vector3d::UnitZ())
                  * AngleAxisd(expectedAngleY,  Vector3d::UnitY())
                  * AngleAxisd(expectedAngleX, Vector3d::UnitX());

        double angleX, angleY, angleZ;
        std::tie(angleX, angleY, angleZ) = findCardanAngles(rotation);
        REQUIRE(std::fabs(expectedAngleX - angleX) < 1e-2);
        REQUIRE(std::fabs(expectedAngleY - angleY) < 1e-2);
        REQUIRE(std::fabs(expectedAngleZ - angleZ) < 1e-2);
    }
}

