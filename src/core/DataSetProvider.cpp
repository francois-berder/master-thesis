#include "DataSetProvider.hpp"


DataSetProvider::DataSetProvider(DataSet &dataSet):
WorkProvider(),
m_mutex(),
m_dataSet(dataSet),
m_currentWorkID(0)
{

}

int DataSetProvider::getWork(cv::Mat &leftFrame, cv::Mat &rightFrame)
{
    int workID;

    m_mutex.lock();
    if(static_cast<unsigned int>(m_currentWorkID) >= m_dataSet.getSize())
        workID = -1;
    else
    {
        workID = m_currentWorkID;
        std::pair<cv::Mat, cv::Mat> data = m_dataSet.get(m_currentWorkID);
        ++m_currentWorkID;
        leftFrame = data.first;
        rightFrame = data.second;
    }
    m_mutex.unlock();

    return workID;
}

void DataSetProvider::resetCounter()
{
    m_currentWorkID = 0;
}
