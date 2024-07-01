//
// Created by yosame on 24-6-28.
//

#ifndef INC_3_FEATURELEARNING_FEATURE_H
#define INC_3_FEATURELEARNING_FEATURE_H

#include <string>
#include <vector>
#include "definitions.h"
#include "structs.h"

enum pName{
    val0,sig1,S1,sig2,mu2,Size
};

class Feature {
public:
    // fit
    Feature():mode(Preprocess){};
    void ReadStat(const std::string &filename, bool rev);
    void FitParam(int threads);
    void Output(const std::string &filename);
    // solve
    explicit Feature(const std::string &filename);
    double Estimate_wrap(double val, int id) const;
    std::vector<int> intervals;
private:
    void EstimateUpdate(int id, const double param[]);
    void FindParam(int id);
    void FindPreciseParam(int id, const double step[], const double ori[]);
    void LoadParam(const std::string& filename);
    SolveMode mode;
    //params
    int upLim[MaxMissing+1]{};
    double sigIndex[MaxMissing+1]{}, sig3Index[MaxMissing+1]{}, thirdIndex[MaxMissing+1]{};
    std::vector<double> stat[MaxMissing+1];
    double params[MaxMissing+1][Size]{},
            cache[MaxMissing+1][Size]{},
            loss[MaxMissing+1]{},
            prep[MaxMissing+1][8192]{};
};


#endif//INC_3_FEATURELEARNING_FEATURE_H
