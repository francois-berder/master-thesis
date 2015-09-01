#ifndef __WORKCOLLECTOR_HPP__
#define __WORKCOLLECTOR_HPP__

#include <opencv2/core/core.hpp>
#include <vector>
#include <utility>
#include <mutex>
#include <tuple>
#include <Eigen/Dense>

typedef std::vector<std::tuple<double, Eigen::Matrix3d, Eigen::Vector3d>> Result;

class WorkCollector
{
    public :

        WorkCollector();

        void saveResult(const int workID, const Result result);
        void saveToDisk();

        std::vector<std::pair<int, Result>> getResults();

    private :

        std::vector<std::pair<int, Result>> m_results;
        std::mutex m_mutex;
};

#endif /* __WORKCOLLECTOR_HPP__ */

