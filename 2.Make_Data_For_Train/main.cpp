#include "opCSV.h"
#include "traffic.h"
#include "FuncEst.h"
#include "funcIO.h"
#include "maths.h"
#include <iostream>
#include <fstream>
#include <thread>
using namespace std;

std::vector<std::vector<double>> road_vectors(PATH_NUM);
vector<string> addHeader = {"toNode", "greenProb", "timeTo0", "vel"};
vector<string> header;

float CycleTime(long long stamp) {
    if(stamp <= 7200) return (7200 - stamp) / 3600.0;
    if(stamp < 14*3600) return (stamp - 7200) / 3600.0;
    return (86400 + 7200 - stamp) / 3600.0;
}

void TaskTurn(){
    ReadStat("../../train_turn_cnt.txt", true);
    FitParam(true);
    Output(PARAMTURN,true);
    cout<<"Turn: Finish"<<endl;
    ReadStat("../../train_difDist_cnt.txt", false);
    FitParam(false);
    Output(PARAMLEN,false);
}

void TaskData(const string &mode, const TrafficHandler& traffics){
    CSVFile rawTraffic("../../"+mode+"_traffic_data.csv");
    safe_clog(mode+" Read Finish");
    CSVFile dataVel(header);
    vector<pair<float,int>>past;
    for(int i=1;i<rawTraffic.size();++i){
        auto &line=rawTraffic[i], &last=rawTraffic[i-1];
        if(last["traj_id"]!=line["traj_id"]){
            past.clear();
            continue;
        }
        int roadID = last["original_path_id"], toID = last["transition_path_id"];
        long long elapsed;
        double passed, toNodeDist = last["distance"];
        if(last["original_path_id"]!=line["original_path_id"]) {
            elapsed = last["elapsed"];
            if(elapsed==0||elapsed>1024)continue;
            passed = toNodeDist;
            toNodeDist /= 2;
        }
        else{
            elapsed = line["sec"]-last["sec"]+(long long)(line["hour"]-last["hour"]+24)%24*3600;
            if(elapsed==0||elapsed>1024)continue;
            passed = toNodeDist-line["distance"];
            toNodeDist = (toNodeDist+line["distance"])/2;
        }
        double vel = passed/elapsed;
        if(vel>40||vel<-3)continue;
        vector<double>newLine;
        dataVel.append(newLine);
        long long timestamp= last["hour"]*3600+last["sec"]+elapsed/2;
        dataVel.back().push_back(road_vectors[roadID], road_vectors[toID], toNodeDist,
                                 traffics.query(roadID, toID, timestamp, toNodeDist), CycleTime(timestamp), vel<0?0:vel);
        past.emplace_back(passed<0?0:passed,elapsed);
    }
    dataVel.saveTo("../../Intermediate/data_vel_"+mode+".csv");
    safe_clog("Write DataVel.csv Finish");
}

int main(){
    std::vector<std::thread> threads;
    //threads.emplace_back(TaskTurn);

    string pref = "vec";
    for(int i=1;i<=vec_len;++i)header.push_back(pref+to_string(i));
    for(int i=1;i<=vec_len;++i)header.push_back(pref+to_string(i));
    header.insert(header.end(),addHeader.begin(),addHeader.end());
    safe_clog("Start to read files");
    {
        ifstream file(ROADVECTOR);
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
    TrafficHandler traffics("../../train_traffic_data.csv");
    threads.emplace_back(TaskData, "train", traffics);
    threads.emplace_back(TaskData, "valid", traffics);

    for(auto &i:threads)i.join();
    return 0;
}
