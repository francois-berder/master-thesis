#ifndef __CAMERAPARAMETERS_HPP__
#define __CAMERAPARAMETERS_HPP__

#include <opencv2/core/core.hpp>

class CameraParameters
{
    public :

        void load();

        cv::Mat& getLeftCameraMatrix();
        cv::Mat& getRightCameraMatrix();
        cv::Mat& getLeftDistCoeffs();
        cv::Mat& getRightDistCoeffs();

    private :

        cv::Mat m_leftCameraMatrix;
        cv::Mat m_rightCameraMatrix;
        cv::Mat m_leftDistCoeffs;
        cv::Mat m_rightDistCoeffs;
};

#endif /* __CAMERAPARAMETERS_HPP__ */

