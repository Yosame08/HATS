//
// Created by yosame on 24-6-15.
//

#include "predictor.h"
#include "safeIO.h"
#include "general.h"
#include <fstream>
using namespace std;

Predictor::Predictor(Graph *g, Light *l) {
    graph = g;
    light = l;
}

void Predictor::readRoadVectors(const std::string &filename) {
    safeCOUT("Reading Embedded Road Vectors...");
    ifstream file(filename);
    int num_roads;
    file >> num_roads;
    for (int i = 0; i <= min(num_roads, RoadNumber - 1); ++i) {
        int road_id;
        file >> road_id;
        std::vector<float> vector(RoadVecLen);
        for (int j = 0; j < RoadVecLen; ++j) file >> vector[j];
        road_vectors[road_id] = vector;
    }
    file.close();
}

void Predictor::loadModel(const string &filename) {
    safeCOUT("Loading Model...");
    modelVel = torch::jit::load(filename);
}

float Predictor::velPredict(int roadID, int toID, float toNodeDist, long long timestamp, float percent) {
    vector<float> inputs(road_vectors[roadID]);
    inputs.push_back(toNodeDist);
    float roadRatio = 1 - toNodeDist / static_cast<float>(graph->lengthOf(roadID));
    inputs.push_back(std::max(roadRatio, 0.f));
    inputs.push_back(static_cast<float>(light->query(roadID, toID, timestamp % 86400, toNodeDist)));
    inputs.push_back(static_cast<float>(CycleTime(timestamp)));
    inputs.push_back(percent);
    torch::Tensor input = torch::from_blob(inputs.data(), {1, static_cast<int64_t>(inputs.size())}, torch::kFloat);
    torch::Tensor output = modelVel.forward({input}).toTensor();
    auto out = output.item<float>();
    if (out < 0.01) out = 0.01;
    return out;
}
