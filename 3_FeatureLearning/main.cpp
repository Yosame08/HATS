#include <iostream>
#include <cstring>
#include <thread>
#include <atomic>
#include "dataset.h"
#include "feature.h"
#include "safeIO.h"
using namespace std;
atomic<int> failed;
Dataset dataset("../../Intermediate/road_vectors.txt");

void TaskDataset(){
    dataset.LightRead("../../Intermediate/train_preprocess.csv");
    dataset.Convert("../../Intermediate/train_preprocess.csv", "train");
    dataset.Convert("../../Intermediate/valid_preprocess.csv", "valid");
}

void TaskFitting(int threads){
    auto turn = new Feature;
    turn->ReadStat("../../Intermediate/train_turn.stat", false);
    turn->FitParam(threads);
    turn->Output(FeatureOutTurn);
    delete turn;
    safeCOUT("[Fit Parameters] Turn: Finish");

    auto lenPos = new Feature;
    lenPos->ReadStat("../../Intermediate/train_dist.stat", false);
    lenPos->FitParam(threads);
    lenPos->Output(FeatureOutLenPos);
    delete lenPos;
    safeCOUT("[Fit Parameters] Length(Positive): Finish");

    auto lenNeg = new Feature;
    lenNeg->ReadStat("../../Intermediate/train_dist.stat", true);
    lenNeg->FitParam(threads);
    lenNeg->Output(FeatureOutLenNeg);
    delete lenNeg;
    safeCOUT("[Fit Parameters] Length(Negative): Finish");
}

int main(int argc, char* argv[]) {
    int num_threads = 4;
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-th") == 0 && i + 1 < argc) {
            num_threads = std::stoi(argv[i + 1]);
            ++i; // skip next argument
        }
        else cout << "redundant argument: " << argv[i] << endl;
    }
    if(num_threads<=1)num_threads=2;
    std::vector<std::thread> threads;
    threads.emplace_back(TaskDataset);
//    threads.emplace_back(TaskFitting, num_threads);
    for(auto &i: threads)i.join();
    return 0;
}
