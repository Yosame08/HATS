//
// Created by yosame on 24-6-26.
//

#include "dataset.h"
#include "safeIO.h"
#include "general.h"
#include <string>
#include <fstream>
#include <cassert>
using namespace std;

enum{
    RoadFreq = RoadVecLen-4,
    RoadLevel,
    RoadLength,
    RoadOutDeg
};

Dataset::Dataset(const string &vecFile) {
    static const std::vector<std::string> extHeaders = {"toNode", "roadRatio", "greenProb", "time", "journey", "vel"};
    string pref = "vec";
    for(int i=1;i<=RoadVecLen;++i)headers.push_back(pref+to_string(i));
    headers.insert(headers.end(), extHeaders.begin(), extHeaders.end());
    ifstream file(vecFile);
    int num_roads;
    file >> num_roads;
    road_vectors.resize(num_roads+1);
    for (int i = 0; i <= num_roads; ++i) {
        int road_id;
        file >> road_id;
        std::vector<double> vector(RoadVecLen);
        for (int j = 0; j < RoadVecLen; ++j) file >> vector[j];
        road_vectors[road_id]=vector;
    }
    file.close();
}

void Dataset::Convert(const std::string &traceFile, const std::string &m) {
    this->mode = m;
    CSVFile rawTraffic(traceFile);
    safeCOUT(traceFile+" read finish");
    vector<pair<double,double>> timeLen;
    auto info = Dataset::ReadTrajectory(rawTraffic, timeLen);
    safeCOUT(traceFile+" preprocess finish");
    ConvertDataset(info, timeLen, rawTraffic);
    safeCOUT(traceFile+" convert finish");
}

vector<Dataset::Info> Dataset::ReadTrajectory(CSVFile &rawTraffic, vector<pair<double,double>> &timeLen) {
    vector<Info>info;
    double totTM=0, jLen=0;
    for(int i=1;i<rawTraffic.size();++i){
        auto &now=rawTraffic[i], &last=rawTraffic[i-1];
        if(last["trajID"]!=now["trajID"]){
            while(timeLen.size()<last["trajID"])timeLen.emplace_back(0,0);
            timeLen.emplace_back(totTM,jLen);
            totTM=jLen=0;
            continue;
        }
        int lastID = last["originID"], lastToID = last["transitionID"], nowID = now["originID"];
        double elapsed = last["elapsed"];
        if(elapsed<=0){ // output all info read this line
            safeCERR("(Warning) lastID="+to_string(lastID)+", lastToID="+to_string(lastToID)+", nowID="+to_string(nowID)+", elapsed="+to_string(elapsed));
        }
        // Take the midpoint of two pieces of data as one piece of training data
        double lastDist = last["toNode"], nowDist = now["toNode"], passed, vel, toNode;
        if(lastID == nowID)passed = lastDist-nowDist, toNode = (lastDist+nowDist)/2;
        else passed = lastDist, toNode = lastDist/2;
        vel = passed/elapsed;
        double timestamp = last["timestamp"]+elapsed/2;
        if(timestamp>=86400)timestamp-=86400;
        if(vel>=-2 && vel<=40){
            info.push_back(Info{passed, vel, toNode, timestamp, elapsed, (int)last["trajID"], lastID, lastToID, i-1});
        }
//        else{
//            safeCERR("(Warning) vel="+to_string(vel)+", lastDist="+to_string(lastDist)+", nowDist="+to_string(nowDist)+", elapsed="+to_string(elapsed));
//        }
        totTM += elapsed, jLen += passed;
    }
    while(timeLen.size()<rawTraffic.back()["trajID"])timeLen.emplace_back(0,0);
    timeLen.emplace_back(totTM,jLen);
    return std::move(info);
}

void Dataset::ConvertDataset(const vector<Info> &info, const vector<pair<double,double>> &timeLen, CSVFile &rawTraffic) {
    CSVFile dataVel(headers);
    double passed = 0;
    for(int i=0;i<info.size();++i){
        if(i&&info[i].trajID==info[i-1].trajID) { // data augmentation
            vector<double> newL;
            newL.reserve(headers.size());
            auto &line = rawTraffic[info[i].csvID];
            double timestamp = line["timestamp"], toNodeDist = line["toNode"];
            double vel = (info[i - 1].vel * info[i - 1].elapsed + info[i].vel * info[i].elapsed) / (info[i - 1].elapsed + info[i].elapsed);
            assert(vel <= 40);
            passed += info[i - 1].passed / 2;
            newL.insert(newL.end(), road_vectors[info[i].roadID].begin(), road_vectors[info[i].roadID].end());
            newL.push_back(toNodeDist);
            newL.push_back(max(0.0, 1 - toNodeDist / 1000 / road_vectors[info[i].roadID][RoadLength]));
            newL.push_back(light.query(info[i].roadID, info[i].toID, timestamp, toNodeDist));
            newL.push_back(CycleTime(timestamp));
            newL.push_back(passed / timeLen[info[i].trajID].second);
            newL.push_back(vel<0?0:vel);
            dataVel.append(std::move(newL));
        }
        else passed = 0;
        vector<double> newL;
        newL.reserve(headers.size());
        double vel = info[i].vel;
        assert(vel <= 40);
        passed += info[i].passed / 2;
        newL.insert(newL.end(), road_vectors[info[i].roadID].begin(), road_vectors[info[i].roadID].end());
        newL.push_back(info[i].toNode);
        newL.push_back(max(0.0, 1 - info[i].toNode / 1000 / road_vectors[info[i].roadID][RoadLength]));
        newL.push_back(light.query(info[i].roadID, info[i].toID, info[i].timestamp, info[i].toNode));
        newL.push_back(CycleTime(info[i].timestamp));
        newL.push_back(passed / timeLen[info[i].trajID].second);
        newL.push_back(vel<0?0:vel);
        dataVel.append(std::move(newL));
    }
    dataVel.saveTo("../../Intermediate/data_vel_"+mode+".csv");
}

void Dataset::LightRead(const string &traceFile) {
    light.parse(traceFile);
}
