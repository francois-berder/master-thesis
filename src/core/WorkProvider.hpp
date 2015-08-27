#ifndef __WORKPROVIDER_HPP__
#define __WORKPROVIDER_HPP__

#include <opencv2/core/core.hpp>

class WorkProvider
{
    public :

        virtual ~WorkProvider() = default;
        virtual int getWork(cv::Mat &leftFrame, cv::Mat &rightFrame) = 0;

};

#endif /* __WORKPROVIDER_HPP__ */

