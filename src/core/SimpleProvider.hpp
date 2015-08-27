#ifndef __SIMPLEPROVIDER_HPP__
#define __SIMPLEPROVIDER_HPP__

#include <mutex>
#include "WorkProvider.hpp"

class SimpleProvider : public WorkProvider
{
    public :

        SimpleProvider(cv::Mat &leftFrame, cv::Mat &rightFrame);
        virtual int getWork(cv::Mat &leftFrame, cv::Mat &rightFrame);

    private :

        cv::Mat &m_leftFrame;
        cv::Mat &m_rightFrame;
        int m_currentWorkID;
        std::mutex m_mutex;
};

#endif /* __SIMPLEPROVIDER_HPP__ */

