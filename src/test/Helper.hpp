#ifndef __HELPER_HPP__
#define __HELPER_HPP__

#include <vector>
#include <Eigen/Dense>
#include <opengv/relative_pose/CentralRelativeAdapter.hpp>
#include <opengv/relative_pose/methods.hpp>
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/relative_pose/CentralRelativePoseSacProblem.hpp>
#include <opencv2/opencv.hpp>

class Helper
{
    public :

        static Eigen::Vector3d createRandomTranslation();
        static Eigen::Matrix3d createRandomRotation();

        static std::vector<Eigen::Vector2d> getRandomPoints(const unsigned int N, double minX, double minY, double maxX, double maxY);

        static Eigen::Matrix3d getCameraMatrix();

        static std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> getPointsCorrespondencesTsukuba();

        static opengv::transformation_t estimateTransformation(std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> &points, Eigen::Matrix3d &cameraMatrix);
};

#endif /* __HELPER_HPP__ */
