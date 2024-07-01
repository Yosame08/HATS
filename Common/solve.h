//
// Created by yosame on 24-6-15.
//

#ifndef INC_4_SOLVE_SOLVE_H
#define INC_4_SOLVE_SOLVE_H

#include "criterion.h"
#include "definitions.h"
#include "graph.h"
#include "light.h"
#include "predictor.h"
#include "progress.h"
#include "traffic.h"
#include "criterion.h"
#include "feature.h"
#include <atomic>

class Solve {
public:
    explicit Solve(SolveMode m):mode(m) {
        if(mode == Preprocess)criterion = std::make_unique<preCriterion>(PreprocessSigZ);
        else criterion = std::make_unique<solveCriterion>("../../Intermediate/params", "../../Intermediate/ParamTurn.params",
                                                         "../../Intermediate/ParamLenPos.params", "../../Intermediate/ParamLenNeg.params"); // TODO
    }
    void initRoadNet(const std::string &edgeFile, const std::string &typeFile);
    void initTraffic(const std::string &roadSpeedFile);
    void initLight(const std::string &traceFile);
    void emptyTraffic(); // for preprocess to initialize an empty Traffic class
    void initModel(const std::string &vectorFile, const std::string &modelFile);
    void readTraces(const std::string &traceFile, bool fourPerRow, bool fixedInterval);
    Searched findPath(const Status &old, const Candidate &now, const GPS &last, const GPS &curr, double minProb);
    void multiSolve(int threads);
    void pathSearch(int id);
    void modelInterpolation(std::vector<Status> &statuses, int id);
    void linearInterpolation(std::vector<Status> &statuses, int id);
    std::vector<std::pair<float, double>> velSequence(const Path &path, long long timeOld, long long timeNow, double journey, double fullLength, int nxtID);
    void outputRecovery(const std::string &recoveryFile);
    void outputPreprocess(const std::string &preprocessFile);
    void outputMatching(const std::string &matchingFile);

private:
    //algorithm
    std::unique_ptr<Criterion> criterion;
    Graph graph;
    Light light;
    Traces traces;
    Predictor predictor = Predictor(&graph, &light);
    //output
    std::ostringstream recovery[TraceNumber];
    std::ostringstream matching[TraceNumber];
    std::ostringstream preprocess[TraceNumber];
    //functional
    Progress progress;
    //parameters
    const SolveMode mode;
    double roadSpeed[RoadType];
    //methods
    int traceCnt();
    bool solveAssert(int id, bool condition, const std::string &cause);
    double speedOf(int roadID);
    //statistics
    std::vector<double>turnAngles[TraceNumber][MaxMissing+1];
    std::vector<double>difDists[TraceNumber][MaxMissing+1];
    std::vector<double>gpsErrors[TraceNumber];
    std::atomic<double>passLen[7];
    std::atomic<double>passTime[7];
};


#endif//INC_4_SOLVE_SOLVE_H
