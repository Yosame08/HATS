//
// Created by yosame on 24-6-17.
//

#ifndef INC_4_SOLVE_CRITERION_H
#define INC_4_SOLVE_CRITERION_H

#include "definitions.h"
#include "feature.h"
#include <string>
#include <fstream>

class Criterion {
public:
    explicit Criterion(double sig):sigZ(sig){}
    double PtMatchProb(double dist) const;
    virtual double AngleProb(double angle, int time) = 0;
    virtual double DifDistProb(double dif, int time) = 0;
    virtual double SearchDifDistProb(double dif, int time) = 0;
    virtual double MedianSpeedProb(double estTime) = 0;
    virtual double Prob(double passed, double gcLeft, double gcTot, double angle, double estTime, int time, bool inSearching) = 0;

protected:
    double sigZ;
    double timeScale = 1;
};

class preCriterion : public Criterion {
public:
    explicit preCriterion(double sig):Criterion(sig){}
    double AngleProb(double angle, int time) override;
    double DifDistProb(double dif, int time) override;
    double SearchDifDistProb(double dif, int time) override;
    double MedianSpeedProb(double estTime) override;
    double Prob(double passed, double gcLeft, double gcTot, double angle, double estTime, int time, bool inSearching) override;
};

class solveCriterion : public Criterion {
public:
    explicit solveCriterion(const std::string &paramFile, const std::string &paramTurn, const std::string &paramLenPos, const std::string &paramLenNeg):
                            Criterion(0), turn(paramTurn), lenPos(paramLenPos), lenNeg(paramLenNeg) {
        std::ifstream in(paramFile);
        std::string inStr;
        for(int i=1;i<=RoadType*2+1;++i)in>>inStr;
        in >> sigZ;
        in.close();
    };
    double AngleProb(double angle, int time) override;
    double DifDistProb(double dif, int time) override;
    double SearchDifDistProb(double dif, int time) override;
    double MedianSpeedProb(double estTime) override;
    double Prob(double passed, double gcLeft, double gcTot, double angle, double estTime, int time, bool inSearching) override;
private:
    void FindIndex(double time, int &l, int &r) const;
    Feature turn, lenPos, lenNeg;
};

#endif//INC_4_SOLVE_CRITERION_H
