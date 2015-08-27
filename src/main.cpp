#include <fstream>
#include <sstream>
#include <vector>
#include <string>

#include "Logger.hpp"
#include "CameraParameters.hpp"
#include "DataSet.hpp"
#include "DataSetProvider.hpp"
#include "WorkCollector.hpp"
#include "Worker.hpp"


double convertStringToDouble(const std::string &s)
{
    std::stringstream ss;
    ss << s;
    double d;
    ss >> d;
    return d;
}

int main(int argc, char* argv[])
{
    if(argc < 2)
    {
        std::cerr << "No input folder specified." << std::endl;
        return -1;
    }

    std::string inputFolder = argv[1];

    // 1. Load camera parameters
    CameraParameters params;
    params.load();

    // 2. Load data
    DataSet dataSet;
    dataSet.load(inputFolder, params);

    // 3. Create provider and collector
    DataSetProvider provider(dataSet);
    WorkCollector collector;

    unsigned int nbCores = std::thread::hardware_concurrency();

    // 4. Create workers
    std::vector<std::string> algorithms;
    for(int i = 2; i < argc; ++i)
        algorithms.push_back(argv[i]);

    std::vector<Worker> workers;
    for(unsigned int i = 0; i < nbCores; ++i)
        workers.push_back(Worker(i, params, algorithms));

    // 5. Start workers
    for(unsigned int i = 0; i < nbCores; ++i)
        workers[i].start(provider, collector);

    // 6. Wait for completion
    for(unsigned int i = 0; i < nbCores; ++i)
        workers[i].wait();

	return 0;
}

