#include "opCSV.h"
#include "definitions.h"
#include "traffic.h"
#include "FuncEst.h"
#include "funcIO.h"
#include "maths.h"
#include <iostream>
#include <fstream>
#include <thread>
#include <cassert>
#include <algorithm>
using namespace std;

std::vector<std::vector<double>> road_vectors(PATH_NUM);
vector<string> addHeader = {"toNode", "roadRatio", "greenProb", "time", "journey", "vel"};
vector<string> header;

void TaskParam(){
    FunctionFit turn(limTurn);
    turn.ReadStat("../../Intermediate/train_turn_cnt.txt", false);
    turn.FitParam();
    turn.Output(PARAMTURN);
    cout<<"[Fit Parameters] Turn: Finish"<<endl;

    FunctionFit lenPos(limLenPos);
    lenPos.ReadStat("../../Intermediate/train_difDist_cnt.txt", false);
    lenPos.FitParam();
    lenPos.Output(PARAMLENPOS);
    cout<<"[Fit Parameters] Length(Positive): Finish"<<endl;

    FunctionFit lenNeg(limLenNeg);
    lenNeg.ReadStat("../../Intermediate/train_difDist_cnt.txt", true);
    lenNeg.FitParam();
    lenNeg.Output(PARAMLENNEG);
    cout<<"[Fit Parameters] Length(Negative): Finish"<<endl;
}

struct Info{
    double passed, vel, toNode;
    long long timestamp;
    int trajID, roadID, toID, begin, elapsed, csvID;
};

void TaskData(const string &mode, const TrafficHandler& traffics){
    CSVFile rawTraffic("../../Intermediate/"+mode+"_traffic_data.csv");
    safe_clog(mode+" read finish");
    CSVFile dataVel(header);
    vector<pair<int,float>>lasting;
    vector<Info>info;
    int totTM = 0;
    float jLen = 0;
    for(int i=1;i<rawTraffic.size();++i){
        auto &line=rawTraffic[i], &last=rawTraffic[i-1];
        if(last["traj_id"]!=line["traj_id"]){
            assert(lasting.size()<=last["traj_id"]);
            while(lasting.size()<last["traj_id"])lasting.emplace_back(0,0);
            lasting.emplace_back(totTM,jLen);
            totTM = jLen = 0;
            continue;
        }
        int lastID = last["original_path_id"], lastToID = last["transition_path_id"],
                nowID = line["original_path_id"],
                elapsed = last["elapsed"];
        double passed, lastDist = last["distance"], nowDist = line["distance"];
        if(lastID == nowID)passed = lastDist-nowDist, lastDist = (lastDist+nowDist)/2;
        else passed = lastDist, lastDist /= 2;
        double vel = passed/elapsed;

        long long timestamp = last["hour"]*3600+last["sec"]+elapsed/2;
        if(timestamp>=86400)timestamp-=86400;
        if(vel>=-3 && vel<40){
            info.push_back(Info{passed, vel, lastDist, timestamp, (int)last["traj_id"], lastID, lastToID, totTM, elapsed, i-1});
        }
        totTM += elapsed;
        jLen += passed;
    }
    while(lasting.size()<rawTraffic.back()["traj_id"])lasting.emplace_back(0,0);
    lasting.emplace_back(totTM,jLen);
    safe_clog(mode+" preprocess finish");
    vector<double>newLine;
    newLine.reserve(vec_len+addHeader.size());
    float passed = 0;
    for(int i=0;i<info.size();++i){
        if(i&&info[i].trajID==info[i-1].trajID){
            dataVel.append(newLine);
            auto &line = rawTraffic[info[i].csvID];
            long long timestamp = line["hour"]*3600+line["sec"];
            //assert(timestamp<=86400);
            double toNodeDist = line["distance"];
            double vel = (info[i-1].vel*info[i-1].elapsed+info[i].vel*info[i].elapsed)/(double)(info[i-1].elapsed+info[i].elapsed);
            passed += info[i-1].passed;
            dataVel.back().push_back(road_vectors[info[i].roadID], toNodeDist/1000, 1-toNodeDist/1000/road_vectors[info[i].roadID][vec_len-1],
                                     traffics.query(info[i].roadID, info[i].toID, timestamp, toNodeDist),
                                     CycleTime(timestamp), passed/lasting[info[i].trajID].second, vel); // info[i].begin/(double)lasting[info[i].trajID].first
        }
        else passed = 0;
        dataVel.append(newLine);
        double vel = info[i].vel;
        passed += info[i].passed;
        //assert(info[i].timestamp<=86400);
        dataVel.back().push_back(road_vectors[info[i].roadID], info[i].toNode/1000, 1-info[i].toNode/1000/road_vectors[info[i].roadID][vec_len-1],
                                 traffics.query(info[i].roadID, info[i].toID, info[i].timestamp, info[i].toNode),
                                 CycleTime(info[i].timestamp), passed/lasting[info[i].trajID].second,  vel); // (info[i].begin+info[i].elapsed/2.0)/lasting[info[i].trajID].first
    }
    dataVel.saveTo("../../Intermediate/data_vel_"+mode+".csv");
    safe_clog("Write DataVel.csv Finish");
}

int main(){
    std::vector<std::thread> threads;
    threads.emplace_back(TaskParam);

    string pref = "vec";
    for(int i=1;i<=vec_len;++i)header.push_back(pref+to_string(i));
    header.insert(header.end(),addHeader.begin(),addHeader.end());
    safe_clog("Start to read files");
    {
        ifstream file("../../Intermediate/road_vectors.txt");
        int num_roads;
        file >> num_roads;
        for (int i = 0; i <= num_roads; ++i) {
            int road_id;
            file >> road_id;
            std::vector<double> vector(vec_len);
            for (int j = 0; j < vec_len; ++j) file >> vector[j];
            road_vectors[road_id]=vector;

        }
        file.close();
    }
    TrafficHandler traffics("../../Intermediate/train_traffic_data.csv");
    threads.emplace_back(TaskData, "train", traffics);
    threads.emplace_back(TaskData, "valid", traffics);

    for(auto &i:threads)i.join();
    return 0;
}
