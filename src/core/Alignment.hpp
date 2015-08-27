#ifndef __ALIGNMENT_HPP__
#define __ALIGNMENT_HPP__

#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <vector>

class Alignment
{
    public :

        Alignment(Eigen::Matrix3d &leftCameraMatrix, Eigen::Matrix3d &rightCameraMatrix);

        std::tuple<cv::Mat, cv::Mat, Eigen::Matrix3d, Eigen::Matrix3d> alignImages(Eigen::Matrix3d &rotation, Eigen::Vector3d &translation, cv::Mat &leftFrame, cv::Mat &rightFrame);

    private :

        Eigen::Matrix3d findRightHomography(Eigen::Vector3d &rightEpipole);
        Eigen::Matrix3d computeR(Eigen::Vector3d &epipole);
        Eigen::Matrix3d findLeftHomography(Eigen::Matrix3d &rightHomography, Eigen::Matrix3d &rotation, Eigen::Vector3d &translation, std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> &points);
        Eigen::Matrix3d computeM(Eigen::Matrix3d &rotation, Eigen::Vector3d &translation);

        Eigen::Matrix3d &m_leftCameraMatrix;
        Eigen::Matrix3d &m_rightCameraMatrix;
};

#endif /* __ALIGNMENT_HPP__ */

