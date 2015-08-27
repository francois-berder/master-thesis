#include "CameraParameters.hpp"

void CameraParameters::load()
{
    cv::FileStorage leftFile("data/calibration/left_calibration.xml", cv::FileStorage::READ);
    leftFile["cameraMatrix"] >> m_leftCameraMatrix;
    leftFile["distCoeffs"] >> m_leftDistCoeffs;

    cv::FileStorage rightFile("data/calibration/right_calibration.xml", cv::FileStorage::READ);
    rightFile["cameraMatrix"] >> m_rightCameraMatrix;
    rightFile["distCoeffs"] >> m_rightDistCoeffs;
}

cv::Mat& CameraParameters::getLeftCameraMatrix()
{
    return m_leftCameraMatrix;
}

cv::Mat& CameraParameters::getRightCameraMatrix()
{
    return m_rightCameraMatrix;
}

cv::Mat& CameraParameters::getLeftDistCoeffs()
{
    return m_leftDistCoeffs;
}

cv::Mat& CameraParameters::getRightDistCoeffs()
{
    return m_rightDistCoeffs;
}

