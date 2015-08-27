#include <fstream>
#include <sstream>
#include <opencv2/highgui/highgui.hpp>
#include "WorkCollector.hpp"
#include "Logger.hpp"
#include "Utils.hpp"

namespace
{

bool compare(std::pair<int, Result> a, std::pair<int, Result> b)
{
    return a.first < b.first;
}

}

WorkCollector::WorkCollector():
m_results(),
m_mutex()
{
    // reset error file
    std::ofstream errorFile("output/error", std::ofstream::trunc);
}

void WorkCollector::saveResult(const int workID, const Result result)
{
    m_mutex.lock();
    m_results.push_back(std::make_pair(workID, result));
    saveToDisk();
    m_mutex.unlock();
}

void WorkCollector::saveToDisk()
{
    std::ofstream errorFile("output/error");

    std::sort(m_results.begin(), m_results.end(), compare);
    for(auto& result : m_results)
    {
        errorFile << result.first << ' ';
        for(auto& r : result.second)
            errorFile << r << ' ';
        errorFile << '\n';
    }
}

std::vector<std::pair<int, Result>> WorkCollector::getResults()
{
    m_mutex.lock();
    std::vector<std::pair<int, Result>> tmp = m_results;
    m_mutex.unlock();

    return tmp;
}


