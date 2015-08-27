#ifndef __UTILS_HPP__
#define __UTILS_HPP__

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <tuple>
#include <vector>
#include <utility>

Eigen::Matrix3d importCVMatrix(cv::Mat &m);
cv::Mat exportGVMatrix(Eigen::Matrix3d &m);

Eigen::Matrix3d skewSym(Eigen::Vector3d &v);

Eigen::Matrix<double, 3, 4> computeLeftProjectionMatrix();
Eigen::Matrix<double, 3, 4> computeRightProjectionMatrix(Eigen::Matrix3d &rotation, Eigen::Vector3d &translation);

Eigen::Vector3d computeLeftEpipole(Eigen::Matrix3d &leftCameraMatrix, Eigen::Vector3d &translation);
Eigen::Vector3d computeRightEpipole(Eigen::Matrix3d &rightCameraMatrix, Eigen::Matrix3d &rotation, Eigen::Vector3d &translation);

std::tuple<Eigen::Matrix3d, Eigen::Vector3d> extractRotationTranslation(Eigen::Matrix3d &F, Eigen::Matrix3d &leftCameraMatrix, Eigen::Matrix3d &rightCameraMatrix, std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> &points);

double computeReprojectionError(Eigen::Matrix3d &leftCameraMatrix, Eigen::Matrix3d &rightCameraMatrix, Eigen::Matrix3d &rotation, Eigen::Vector3d &translation, std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> &points);

std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> findPoints(cv::Mat &leftFrame, cv::Mat &rightFrame, unsigned int nbFeatures, float distance);

std::tuple<double, double, double> findCardanAngles(Eigen::Matrix3d &rotation);
double convertRadToDeg(double angle);

template<typename U, typename T>
T convertTo(U str)
{
    std::stringstream ss;
    ss << str;
    T value;
    ss >> value;
    return value;
}

template<typename T>
T convertStringTo(std::string &str)
{
    std::stringstream ss;
    ss << str;
    T value;
    ss >> value;
    return value;
}

template<typename T>
T convertStringTo(std::string &&str)
{
    std::stringstream ss;
    ss << str;
    T value;
    ss >> value;
    return value;
}
#endif /* __UTILS_HPP__ */

