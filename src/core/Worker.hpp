#ifndef __WORKER_HPP__
#define __WORKER_HPP__

#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <vector>
#include <utility>
#include <string>
#include <thread>
#include "WorkProvider.hpp"
#include "WorkCollector.hpp"
#include "CameraParameters.hpp"

class Worker
{
    public :

        Worker(const unsigned int ID, CameraParameters &params, std::vector<std::string> &algorithms);

        void start(WorkProvider &provider, WorkCollector &collector);
        void stop();
        void wait();

        unsigned int getID() const;
        bool isRunning() const;

    private :

        void run(WorkProvider &provider, WorkCollector &collector);
        std::vector<double> doWork(const int workID, cv::Mat &leftFrame, cv::Mat &rightFrame);
        std::vector<std::tuple<Eigen::Matrix3d, Eigen::Vector3d, double>> computeFundamentalMatrix(cv::Mat &leftFrame, cv::Mat &rightFrame);

        std::tuple<Eigen::Matrix3d, Eigen::Vector3d> applyRANSACfromOpenGV(cv::Mat &leftFrame, cv::Mat &rightFrame, std::vector<std::string> &algoParams);
        std::tuple<Eigen::Matrix3d, Eigen::Vector3d> applyModel(Eigen::Matrix3d &rotation, Eigen::Vector3d &translation);
        std::tuple<Eigen::Matrix3d, Eigen::Vector3d> applyNonLinearMinimization(Eigen::Matrix3d &initialRotation, Eigen::Vector3d &initialTranslation, cv::Mat &leftFrame, cv::Mat &rightFrame, std::vector<std::string> &algoParams);

        std::tuple<cv::Mat, cv::Mat, Eigen::Matrix3d, Eigen::Matrix3d> alignFrames(const int workID, Eigen::Matrix3d &rotation, Eigen::Vector3d &translation, cv::Mat &leftFrame, cv::Mat &rightFrame);
        cv::Mat computeDisparityMap(const int workID, cv::Mat &leftAligned, cv::Mat &rightAligned);
        cv::Mat computeDepthMap(const int workID, Eigen::Matrix3d &rotation, cv::Mat &disparityMap);
        void computePointCloud(const int workID, cv::Mat &leftFrame, Eigen::Matrix3d &Hl, cv::Mat &depthMap);

        double modelFX(double x);
        double modelFY(double y);
        double modelFS(double s);

        const unsigned int m_ID;
        bool m_isRunning;
        std::thread m_thread;

        Eigen::Matrix3d m_leftCameraMatrix;
        Eigen::Matrix3d m_rightCameraMatrix;
        std::vector<std::string> m_algorithms;
};

#endif /* __WORKER_HPP__ */

