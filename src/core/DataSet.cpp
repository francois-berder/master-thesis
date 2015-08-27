#include <sstream>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "DataSet.hpp"
#include "Logger.hpp"


namespace
{

int extractNumber(const std::string &str)
{
    std::stringstream ss;
    ss << str.substr(str.find_last_of('_') + 1, str.find_last_of('.'));
    int n;
    ss >> n;
    return n;
}

bool compareStr(std::string a, std::string b)
{
    return extractNumber(a) < extractNumber(b);
}

}

void DataSet::load(const std::string &folder, CameraParameters &params, const int limit)
{
    LOG("Loading data");

    std::ifstream file(folder + "/images.xml");
    std::string line;

    // Skip first three lines
    std::getline(file, line);
    std::getline(file, line);
    std::getline(file, line);

    std::vector<std::string> leftFiles, rightFiles;
    goto startLoop;
    do
    {
		if(line.rfind("left_") != std::string::npos)
	        leftFiles.push_back(line);
		else if(line.rfind("right_") != std::string::npos)
	        rightFiles.push_back(line);
        startLoop: std::getline(file, line);
    }while(line != "</images>");
    
    std::sort(leftFiles.begin(), leftFiles.end(), compareStr);
    std::sort(rightFiles.begin(), rightFiles.end(), compareStr);
    unsigned int n = std::min(leftFiles.size(), rightFiles.size());
    if(limit != -1 && static_cast<int>(n) > limit)
        n = limit;

    m_data.clear();
    m_data.resize(n);
    for(unsigned int i = 0; i < n; ++i)
    {
        LOG(".");
        cv::Mat leftFrame = cv::imread(leftFiles[i]);
        cv::Mat rightFrame = cv::imread(rightFiles[i]);

        cv::Mat undistortedLeftFrame, undistortedRightFrame;
        cv::undistort(leftFrame, undistortedLeftFrame, params.getLeftCameraMatrix(), params.getLeftDistCoeffs());
        cv::undistort(rightFrame, undistortedRightFrame, params.getRightCameraMatrix(), params.getRightDistCoeffs());

        m_data[i] = std::make_pair(undistortedLeftFrame, undistortedRightFrame);
    }

    LOG("\n");
}

std::pair<cv::Mat, cv::Mat> DataSet::get(const unsigned int i) const
{
    return m_data[i];
}

unsigned int DataSet::getSize() const
{
    return m_data.size();
}

