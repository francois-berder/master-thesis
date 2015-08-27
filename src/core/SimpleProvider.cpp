#include "SimpleProvider.hpp"

SimpleProvider::SimpleProvider(cv::Mat &leftFrame, cv::Mat &rightFrame):
WorkProvider(),
m_leftFrame(leftFrame),
m_rightFrame(rightFrame),
m_currentWorkID(0),
m_mutex()
{

}

int SimpleProvider::getWork(cv::Mat &leftFrame, cv::Mat &rightFrame)
{
    int workID;

    m_mutex.lock();
    if(m_currentWorkID > 0)
        workID = -1;
    else
    {
        workID = m_currentWorkID;
        ++m_currentWorkID;
        leftFrame = m_leftFrame;
        rightFrame = m_rightFrame;
    }
    m_mutex.unlock();

    return workID;
}
