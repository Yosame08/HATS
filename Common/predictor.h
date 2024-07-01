//
// Created by yosame on 24-6-15.
//

#ifndef INC_4_SOLVE_PREDICTOR_H
#define INC_4_SOLVE_PREDICTOR_H

#include "definitions.h"
#include "graph.h"
#include "light.h"
#include <string>
#include <torch/script.h>
#include <vector>

class Predictor {
public:
    Predictor(Graph *g, Light *l);
    void readRoadVectors(const std::string &filename);
    void loadModel(const std::string &filename);
    float velPredict(int roadID, int toID, float toNodeDist, long long timestamp, float percent);

private:
    std::vector<float> road_vectors[RoadNumber];
    torch::jit::Module modelVel;
    Graph *graph;
    Light *light;
};


#endif//INC_4_SOLVE_PREDICTOR_H
