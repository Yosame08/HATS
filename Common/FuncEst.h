#ifndef FUNCEST_H
#define FUNCEST_H

#include <string>
#include <vector>
#include "definitions.h"

class FunctionFit{
public:
    std::vector<int> times;

    enum pName{
        sig1,S1,sig2,mu2,Size
    };
    // use this to fit parameters
    explicit FunctionFit(int limit): upLim(limit){}
    // use this to load parameters
    explicit FunctionFit(int limit, const std::string &filename): upLim(limit){
        LoadParam(filename);
    }
    // Set rev to true to read negative values
    void ReadStat(const std::string &filename, bool rev);
    void FitParam();
    void Output(const std::string &filename);
    void LoadParam(const std::string& filename);
    double Estimate_wrap(double val, int id) const;

private:
    int upLim{};
    std::vector<double> stat[mxMissing + 1];
    double params[mxMissing + 1][Size]{}, cache[mxMissing + 1][Size]{}, loss[mxMissing + 1]{}, prep[mxMissing + 1][10000];

    void FindParam(int id);
    void FindPreciseParam(int id, double param[], const double step[], const double ori[]);
    void EstiUpdate(int id, const double param[]);
    static double Estimate(double x, const double param[], const double cache[]);
};

#endif //FUNCEST_H
