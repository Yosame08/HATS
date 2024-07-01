//
// Created by yosame on 24-6-17.
//

#include "criterion.h"
#include "definitions.h"
#include "safeIO.h"
#include <cmath>
using namespace std;

double solveCriterion::AngleProb(double angle, int time) {
    if(isnan(angle))throw runtime_error("Invalid angle! Nan in AngleProb");
    int l, r;
    FindIndex(time, l, r);
    if(l == r)return turn.Estimate_wrap(angle * 180 / M_PI, l);
    return turn.Estimate_wrap(angle * 180 / M_PI, l) * (r - time) / (r - l)
           + turn.Estimate_wrap(angle * 180 / M_PI, r) * (time - l) / (r - l);
}

double solveCriterion::DifDistProb(double dif, int time) {
    if(isnan(dif)) throw runtime_error("Invalid dif! Nan in DifDistProb");
    const auto &use = dif < 0 ? lenNeg : lenPos;
    int l, r;
    FindIndex(time, l, r);
    if(l == r)return use.Estimate_wrap(dif, l);
    return use.Estimate_wrap(dif, l) * (r - time) / (r - l)
           + use.Estimate_wrap(dif, r) * (time - l) / (r - l);
}

double solveCriterion::SearchDifDistProb(double dif, int time) {
    if(isnan(dif)) throw runtime_error("Invalid dif! Nan in SearchDifDistProb");
    if(dif < 0) return 1;
    return DifDistProb(dif, time);
}

double solveCriterion::MedianSpeedProb(double estTime) {
    return 1 / max(estTime, 1.0);
}

double solveCriterion::Prob(double passed, double gcLeft, double gcTot, double angle, double estTime, int time, bool inSearching) {
    double ret = AngleProb(angle, time) * MedianSpeedProb(estTime);
    return ret * (inSearching ? SearchDifDistProb(passed + gcLeft - gcTot, time) : DifDistProb(passed - gcTot, time));
}

void solveCriterion::FindIndex(double time, int &l, int &r) const {
    const auto &intervals = turn.intervals;
    int iTime = static_cast<int>(round(time));
    auto f = lower_bound(intervals.begin(), intervals.end(), iTime);
    if(f == intervals.end()) l = r = static_cast<int>(intervals.size()) - 1;
    else if(f == intervals.begin() && iTime < *f) l = r = 0;
    else if(iTime == *f) l = r = static_cast<int>(f - intervals.begin());
    else r = static_cast<int>(f - intervals.begin()), l = r - 1;
}

double preCriterion::DifDistProb(double dif, int time) {
    return exp(-abs(dif) / PreprocessBeta) / PreprocessBeta;
}

double preCriterion::AngleProb(double angle, int time) {
    safeCERR("AngleProb should not be called in preprocessing stage");
    exit(-1);
}

double preCriterion::SearchDifDistProb(double dif, int time) {
    safeCERR("SearchDifDistProb should not be called in preprocessing stage");
    exit(-1);
}

double preCriterion::MedianSpeedProb(double estTime) {
    safeCERR("MedianSpeedProb should not be called in preprocessing stage");
    exit(-1);
}

double preCriterion::Prob(double passed, double gcLeft, double gcTot, double angle, double estTime, int time, bool inSearching) {
    return DifDistProb(passed + gcLeft - gcTot, time);
}

double Criterion::PtMatchProb(double dist) const {
    static const double coefficient = 1 / (sigZ * sqrt(M_PI * 2));
    double param = dist / sigZ;
    return exp(-(param * param * 0.5)) * coefficient;
}