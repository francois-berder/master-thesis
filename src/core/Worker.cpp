#include <sstream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <tuple>
#include <opengv/relative_pose/CentralRelativeAdapter.hpp>
#include <opengv/relative_pose/methods.hpp>
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/relative_pose/CentralRelativePoseSacProblem.hpp>

#include "Worker.hpp"
#include "Logger.hpp"
#include "Utils.hpp"
#include "Alignment.hpp"

using namespace opengv;
using namespace Eigen;
using namespace cv;

namespace
{

// Copied from http://stackoverflow.com/questions/236129/split-a-string-in-c
std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems)
{
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}

std::vector<std::string> split(const std::string &s, char delim)
{
    std::vector<std::string> elems;
    split(s, delim, elems);
    return elems;
}

void enhanceDisparityMap(Mat &disparityMap, Mat &leftFrame, Mat &rightFrame)
{
    unsigned char *ldata = (unsigned char*)leftFrame.data;
    unsigned char *rdata = (unsigned char*)rightFrame.data;

    for(unsigned int row = 0; row < 480; ++row)
        for(unsigned int col = 0; col < 640; ++col)
        {
            if((ldata[0] == 0 && ldata[1] == 0 && ldata[2] == 0)
            || (rdata[0] == 0 && rdata[1] == 0 && rdata[2] == 0))
                disparityMap.at<short>(row, col) = 0;

            ldata += 3;
            rdata += 3;
        }
}

void normalizeAndSave(const std::string& fileName, Mat &frame)
{
    Mat output;
    normalize(frame, output, 0, 255, cv::NORM_MINMAX, CV_8U);
    imwrite(fileName, output);
}

}

Worker::Worker(const unsigned int ID, CameraParameters &params, std::vector<std::string> &algorithms):
m_ID(ID),
m_isRunning(false),
m_thread(),
m_leftCameraMatrix(),
m_rightCameraMatrix(),
m_algorithms(algorithms)
{
    m_leftCameraMatrix = importCVMatrix(params.getLeftCameraMatrix());
    m_rightCameraMatrix = importCVMatrix(params.getRightCameraMatrix());
}

void Worker::start(WorkProvider &provider, WorkCollector &collector)
{
    if(m_isRunning)
        return;

    LOG("Starting worker " << m_ID << "\n");

    m_isRunning = true;
    std::thread t(&Worker::run, this, std::ref(provider), std::ref(collector));
    m_thread.swap(t);
}

void Worker::stop()
{
    LOG("Stopping worker " << m_ID << "\n");

    m_isRunning = false;
    m_thread.join();
}

void Worker::wait()
{
    LOG("Waiting for worker " << m_ID << "\n");

    m_thread.join();
}

unsigned int Worker::getID() const
{
    return m_ID;
}

bool Worker::isRunning() const
{
    return m_isRunning;
}

void Worker::run(WorkProvider &provider, WorkCollector &collector)
{
    if(!m_isRunning)
        return;

    cv::Mat leftFrame, rightFrame;
    int workID = provider.getWork(leftFrame, rightFrame);
    while(m_isRunning && workID != -1)
    {
        LOG("worker " << m_ID << " doing work " << workID << "\n");
        std::vector<std::tuple<double, Matrix3d, Vector3d>> result = doWork(workID, leftFrame, rightFrame);
        collector.saveResult(workID, result);
        if(m_isRunning)
            workID = provider.getWork(leftFrame, rightFrame);
    }

    m_isRunning = false;
}

std::vector<std::tuple<double, Matrix3d, Vector3d>> Worker::doWork(const int workID, cv::Mat &leftFrame, cv::Mat &rightFrame)
{
    std::vector<std::tuple<double, Matrix3d, Vector3d>> transformations = computeFundamentalMatrix(leftFrame, rightFrame);

    std::tuple<double, Matrix3d, Vector3d> bestTransformation = *std::min_element(transformations.begin(), transformations.end(), [](std::tuple<double, Matrix3d, Vector3d> &a, std::tuple<double, Matrix3d, Vector3d> &b) { return std::get<0>(a) < std::get<0>(b); });

    Matrix3d rotation = std::get<1>(bestTransformation);
    Vector3d translation = std::get<2>(bestTransformation);
    Mat leftAligned, rightAligned;
    Matrix3d Hl;
    std::tie(leftAligned, rightAligned, Hl, std::ignore) = alignFrames(workID, rotation, translation, leftFrame, rightFrame);

    Mat disparityMap = computeDisparityMap(workID, leftAligned, rightAligned);
    Mat depthMap = computeDepthMap(workID, rotation, disparityMap);
    computePointCloud(workID, leftFrame, Hl, depthMap);

    return transformations;
}

std::vector<std::tuple<double, Matrix3d, Vector3d>> Worker::computeFundamentalMatrix(cv::Mat &leftFrame, cv::Mat &rightFrame)
{
    std::vector<std::pair<Vector2d, Vector2d>> points = findPoints(leftFrame, rightFrame, 2000, 40.f);

    std::vector<std::tuple<double, Matrix3d, Vector3d>> transformations;
    for(auto& s : m_algorithms)
    {
        Matrix3d rotation;
        Vector3d translation;

        if(s.find("opengv-ransac") == 0)
        {
            std::vector<std::string> algoParams = split(s.substr(std::string("opengv-ransac").size(), s.size()), '-');
            std::tie(rotation, translation) = applyRANSACfromOpenGV(leftFrame, rightFrame, algoParams);
        }
        else if(s.find("model") == 0)
        {
            std::vector<std::string> algoParams = split(s.substr(std::string("model").size(), s.size()), '-');
            unsigned int reverseIndex = convertStringTo<unsigned int>(algoParams[0]);

            Matrix3d initialRotation = std::get<1>(transformations[transformations.size() - 1 - reverseIndex]);
            Vector3d initialTranslation =  std::get<2>(transformations[transformations.size() - 1 - reverseIndex]);
            algoParams.erase(algoParams.begin());
            std::tie(rotation, translation) = applyModel(initialRotation, initialTranslation, algoParams);
        }
        else if(s.find("opengv-minimization") == 0)
        {
            std::vector<std::string> algoParams = split(s.substr(std::string("opengv-minimization").size(), s.size()), '-');
            unsigned int reverseIndex = convertStringTo<unsigned int>(algoParams[0]);

            Matrix3d initialRotation = std::get<1>(transformations[transformations.size() - 1 - reverseIndex]);
            Vector3d initialTranslation =  std::get<2>(transformations[transformations.size() - 1 - reverseIndex]);
            algoParams.erase(algoParams.begin());
            std::tie(rotation, translation) = applyNonLinearMinimization(initialRotation, initialTranslation, leftFrame, rightFrame, algoParams);
        }

        double error = computeReprojectionError(m_leftCameraMatrix, m_rightCameraMatrix, rotation, translation, points);

        transformations.push_back(std::make_tuple(error, rotation, translation));
    }


    return transformations;
}

std::tuple<Matrix3d, Vector3d> Worker::applyRANSACfromOpenGV(cv::Mat &leftFrame, cv::Mat &rightFrame, std::vector<std::string> &algoParams)
{
    std::string algoName = algoParams[0];
    unsigned int nbFeatures = convertStringTo<unsigned int>(algoParams[1]);
    float distance = convertStringTo<float>(algoParams[2]);

    std::vector<std::pair<Vector2d, Vector2d>> points = findPoints(leftFrame, rightFrame, nbFeatures, distance);

    bearingVectors_t xls, xrs;
    for(auto &e : points)
    {
        Vector3d xl(e.first(0), e.first(1), 1.);
        Vector3d xr(e.second(0), e.second(1), 1.);

        xl = m_leftCameraMatrix.inverse() * xl;
        xr = m_rightCameraMatrix.inverse() * xr;

        xl /= xl.norm();
        xr /= xr.norm();

        xls.push_back(xl);
        xrs.push_back(xr);
    }

    relative_pose::CentralRelativeAdapter adapter(xls, xrs);
    
    sac::Ransac<sac_problems::relative_pose::CentralRelativePoseSacProblem> ransac;

    opengv::sac_problems::relative_pose::CentralRelativePoseSacProblem::Algorithm algo;
    if(algoName == "stewenius")
        algo = sac_problems::relative_pose::CentralRelativePoseSacProblem::STEWENIUS;
    else if(algoName == "nister")
        algo = sac_problems::relative_pose::CentralRelativePoseSacProblem::NISTER;
    else if(algoName == "sevenpt")
        algo = sac_problems::relative_pose::CentralRelativePoseSacProblem::SEVENPT;
    else if(algoName == "eightpt")
        algo = sac_problems::relative_pose::CentralRelativePoseSacProblem::EIGHTPT;

    boost::shared_ptr<sac_problems::relative_pose::CentralRelativePoseSacProblem>
    relposeproblem_ptr(new sac_problems::relative_pose::CentralRelativePoseSacProblem(
                       adapter,
                       algo));

    ransac.sac_model_ = relposeproblem_ptr;
    ransac.threshold_ = 1.0 - cos(atan(std::sqrt(.5)*0.5/m_leftCameraMatrix(0,0)));
    ransac.max_iterations_ = 100000;
    ransac.computeModel();
    transformation_t best_transformation = ransac.model_coefficients_;

    Matrix3d rotation = best_transformation.block(0,0,3,3);
    Vector3d translation = best_transformation.col(3);
    translation /= translation.norm();

    return std::make_tuple(rotation, translation);
}

std::tuple<Matrix3d, Vector3d> Worker::applyModel(Matrix3d &initialRotation, Vector3d &initialTranslation, std::vector<std::string> &algoParams)
{
    double angleX, angleY, angleZ;
    std::tie(angleX, angleY, angleZ) = findCardanAngles(initialRotation);

    double x, y;

    if(algoParams[0] == "linear")
    {
        x = linearModelFX(angleZ);
        y = linearModelFY(angleZ);
    }
    else if(algoParams[0] == "secondOrder")
    {
        x = secondOrderModelFX(angleZ);
        y = secondOrderModelFY(angleZ);
    }
    else if(algoParams[0] == "mixed")
    {
        x = secondOrderModelFX(angleZ);
        y = linearModelFY(angleZ);
    }

    double l = x*x + y*y;
    if(l > 1.)
    {
        x /= l;
        y /= l;
    }
    double z = std::sqrt(1 - x*x - y*y);

    Vector3d translation(x,y,z);
    Matrix3d rotation;
    rotation = AngleAxisd(angleZ, Vector3d::UnitZ());

    return std::make_tuple(initialRotation, translation);
}

std::tuple<Matrix3d, Vector3d> Worker::applyNonLinearMinimization(Matrix3d &initialRotation, Vector3d &initialTranslation, cv::Mat &leftFrame, cv::Mat &rightFrame, std::vector<std::string> &algoParams)
{
    unsigned int nbFeatures = convertStringTo<unsigned int>(algoParams[0]);
    float distance = convertStringTo<float>(algoParams[1]);

    std::vector<std::pair<Vector2d, Vector2d>> points = findPoints(leftFrame, rightFrame, nbFeatures, distance);

    bearingVectors_t xls, xrs;
    for(auto &e : points)
    {
        Vector3d xl(e.first(0), e.first(1), 1.);
        Vector3d xr(e.second(0), e.second(1), 1.);

        xl = m_leftCameraMatrix.inverse() * xl;
        xr = m_rightCameraMatrix.inverse() * xr;

        xl /= xl.norm();
        xr /= xr.norm();

        xls.push_back(xl);
        xrs.push_back(xr);
    }

    relative_pose::CentralRelativeAdapter adapter(xls, xrs, initialTranslation, initialRotation);
    transformation_t nonlinear_transformation = relative_pose::optimize_nonlinear(adapter);

    Matrix3d rotation = nonlinear_transformation.block(0,0,3,3);
    Vector3d translation = nonlinear_transformation.col(3);
    translation /= translation.norm();

    return std::make_tuple(rotation, translation);
}

std::tuple<Mat, Mat, Matrix3d, Matrix3d> Worker::alignFrames(const int workID, Matrix3d &rotation, Vector3d &translation, Mat &leftFrame, Mat &rightFrame)
{
    Alignment alignmentAlgo(m_leftCameraMatrix, m_rightCameraMatrix);

    Mat leftAligned, rightAligned;
    Matrix3d Hl, Hr;
    std::tie(leftAligned, rightAligned, Hl, Hr) = alignmentAlgo.alignImages(rotation, translation, leftFrame, rightFrame);
    std::stringstream lss;
    lss << "output/alignment/left_" << workID << ".png";
    std::stringstream rss;
    rss << "output/alignment/right_" << workID << ".png";
    imwrite(lss.str(), leftAligned);
    imwrite(rss.str(), rightAligned);

    return std::make_tuple(leftAligned, rightAligned, Hl, Hr);
}

Mat Worker::computeDisparityMap(const int workID, Mat &leftAligned, Mat &rightAligned)
{
    CvStereoBMState currentState =  
    {
        CV_STEREO_BM_NORMALIZED_RESPONSE,
        5,
        0,
        9,
        0,
        32,
        0,
        0,
        0,
        0
    };

    cv::StereoSGBM sgbm(currentState.minDisparity,
                    currentState.numberOfDisparities,
                    currentState.SADWindowSize,
                    8*3*currentState.SADWindowSize*currentState.SADWindowSize,
                    32*3*currentState.SADWindowSize*currentState.SADWindowSize,
                    3,
                    currentState.preFilterCap,
                    currentState.uniquenessRatio,
                    currentState.speckleWindowSize,
                    currentState.speckleRange,
                    true);

    Mat disparity(480, 640, CV_16S);
    sgbm(leftAligned, rightAligned, disparity);

    enhanceDisparityMap(disparity, leftAligned, rightAligned);

    std::stringstream fileName;
    fileName << "output/disparity/disparity_" << workID << ".png";
    normalizeAndSave(fileName.str(), disparity);

    return disparity;
}

Mat Worker::computeDepthMap(const int workID, Matrix3d &rotation, Mat &disparityMap)
{
    double angleX, angleY, angleZ;
    std::tie(angleX, angleY, angleZ) = findCardanAngles(rotation);

    const double baseline = 122.4 * modelFS(angleZ);

    // focal length computed from the camera specifications 
    // instead of using estimated focal length;
    const double focalLength = 768.;//m_leftCameraMatrix(0, 0);

    Mat depthMap(480, 640, CV_64F);
    for(unsigned int row = 0; row < 480; ++row)
    {
        for(unsigned int col = 0; col < 640; ++col)
        {
            double disparity = static_cast<double>(disparityMap.at<short>(row, col)) / 16.;
            double depth = -1.;
            if(disparity != 0. && disparity != -1.)
                depth = (focalLength * baseline) / disparity;

            depthMap.at<double>(row, col) = depth;
        }
    }

    std::stringstream fileName;
    fileName << "output/depth/depth_" << workID << ".png";
    normalizeAndSave(fileName.str(), depthMap);

    return depthMap;
}

void Worker::computePointCloud(const int workID, Mat &leftFrame, Matrix3d &Hl, Mat &depthMap)
{
    cv::Mat leftH = exportGVMatrix(Hl);
    std::stringstream fileName;
    fileName << "output/pointcloud/pointcloud_" << workID;
    std::ofstream file(fileName.str());

    for(unsigned int row = 0; row < 480; ++row)
    {
        for(unsigned int col = 0; col < 640; ++col)
        {
            // Get depth
            double z = depthMap.at<double>(row, col);
            if(z == -1.)
                continue;

            Mat p(Matx31d(col, row, 1.));
            Mat dst = leftH.inv() * p;
            dst /= dst.at<double>(2);
            if(dst.at<double>(0) < 0. || dst.at<double>(0) > 640.)
                continue;
            if(dst.at<double>(1) < 0. || dst.at<double>(1) > 480.)
                continue;

            // Get color
            unsigned int x = static_cast<unsigned int>(dst.at<double>(0));
            unsigned int y = static_cast<unsigned int>(dst.at<double>(1));
            cv::Vec3b pixel = leftFrame.at<cv::Vec3b>(y, x);

            double X = (static_cast<double>(x) - 319.5);
            double Y = (static_cast<double>(480-y) - 219.5);

            // Save point
            file << X << ' ' << Y  << ' ' << z/178.6 << ' ' << static_cast<unsigned int>(pixel.val[2]) << ' ' << static_cast<unsigned int>(pixel.val[1]) << ' ' << static_cast<unsigned int>(pixel.val[0]) << '\n';
        }
    }
}

double Worker::linearModelFX(double x)
{
    return 1.00299382 - 0.05123665 * x;
}

double Worker::linearModelFY(double y)
{
    return 0.02351091 + 0.44311857 * y;
}

double Worker::secondOrderModelFX(double x)
{
    return 1.00086285 - 0.02611081 * x - 0.05968082 * x * x;
}

double Worker::secondOrderModelFY(double y)
{
    return 0.01318154 + 0.5649101 * y -0.28928852 * y * y;
}

double Worker::modelFS(double s)
{
    return 0.94433258 + 0.9565365 * s;
}

