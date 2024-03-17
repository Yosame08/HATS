#include "definitions.h"
#include "traffic.h"
#include "maths.h"
#include <torch/script.h>
#include <vector>
#include <utility>
#include <iostream>
using std::vector;
using std::pair;
using std::cout;
using std::endl;

torch::jit::Module moduleVel = torch::jit::load("../../Intermediate/model_vel.pt");
TrafficHandler traffics("../../Intermediate/train_traffic_data.csv");

extern vector<float> road_vectors[PATH_NUM];
float VelPrediction(int roadID, int toID, float toNodeDist, long long timestamp){
    vector<float>inputs(road_vectors[roadID]);
    for(auto x:road_vectors[toID])inputs.push_back(x);
    inputs.push_back(toNodeDist);
    inputs.push_back(traffics.query(roadID, toID, timestamp%86400, toNodeDist));
    inputs.push_back(CycleTime(timestamp%86400));
    torch::Tensor input = torch::from_blob(inputs.data(), {1, static_cast<int64_t>(inputs.size())}, torch::kFloat);
    torch::Tensor output = moduleVel.forward({input}).toTensor();
    auto out = output.item<float>();
    if(out<0.01)out=0.01;
    return out;
}
