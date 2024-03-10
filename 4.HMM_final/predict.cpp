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

torch::jit::Module moduleVel = torch::jit::load("../../model_vel.pt");
TrafficHandler traffics;
bool init(){
    traffics.init("../../train_traffic_data.csv");
    return false;
}
bool inited = init();

extern vector<float> road_vectors[PATH_NUM];
float VelPrediction(int roadID, int toID, float toNodeDist, float vel, long long timestamp){
    int hour=(int(timestamp%86400)/3600+TIMEZONE)%24;
    vector<float>inputs(road_vectors[roadID]);
    inputs[vec_len] = toNodeDist;
    inputs[vec_len+1] = (float)distToTwo(hour);
    inputs[vec_len+2] = (float)traffics.query(roadID, toID, timestamp%86400);
    inputs[vec_len+3] = vel;
    torch::Tensor input = torch::from_blob(inputs.data(), {1, static_cast<int64_t>(inputs.size())}, torch::kFloat);
    torch::Tensor output = moduleVel.forward({input}).toTensor();
    return output.item<float>();
}
