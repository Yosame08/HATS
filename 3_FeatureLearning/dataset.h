//
// Created by yosame on 24-6-26.
//

#ifndef INC_3_FEATURELEARNING_DATASET_H
#define INC_3_FEATURELEARNING_DATASET_H

#include <vector>
#include <string>
#include "definitions.h"
#include "csv.h"
#include "light.h"

class Dataset {
public:
    explicit Dataset(const std::string &vecFile);
    void LightRead(const std::string &traceFile);
    void Convert(const std::string &traceFile, const std::string &m);
private:
    struct Info{
        double passed, vel, toNode, timestamp, elapsed;
        int trajID, roadID, toID, csvID;
    };
    static std::vector<Info> ReadTrajectory(CSVFile &rawTraffic, std::vector<std::pair<double,double>> &timeLen);
    void ConvertDataset(const std::vector<Info> &info, const std::vector<std::pair<double,double>> &timeLen, CSVFile &rawTraffic);
    std::vector<std::string> headers;
    std::vector<std::vector<double>> road_vectors;
    Light light;
    std::string mode;
};


#endif//INC_3_FEATURELEARNING_DATASET_H
