#ifndef __DATASET_HPP__
#define __DATASET_HPP__

#include <opencv2/core/core.hpp>
#include <vector>
#include <utility>
#include <string>
#include "CameraParameters.hpp"

class DataSet
{
    public :

        DataSet() = default;

        void load(const std::string &folder, CameraParameters &params, const int limit = -1);

        std::pair<cv::Mat, cv::Mat> get(const unsigned int i) const;
        unsigned int getSize() const;

    private :

        DataSet(const DataSet &ds) = default;
        DataSet& operator=(const DataSet &ds) = default;

        std::vector<std::pair<cv::Mat, cv::Mat>> m_data;
};

#endif /* __DATASET_HPP__ */

