#ifndef __DATASETPROVIDER_HPP__
#define __DATASETPROVIDER_HPP__

#include <opencv2/core/core.hpp>
#include <mutex>
#include "DataSet.hpp"
#include "WorkProvider.hpp"


class DataSetProvider : public WorkProvider
{
    public :

        DataSetProvider(DataSet &dataSet);
        virtual int getWork(cv::Mat &leftFrame, cv::Mat &rightFrame);
        void resetCounter();

    private :

        std::mutex m_mutex;
        DataSet &m_dataSet;
        int m_currentWorkID;
};

#endif /* __WORKPROVIDER_HPP__ */

