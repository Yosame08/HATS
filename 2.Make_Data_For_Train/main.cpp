#include "opCSV.h"
#include "traffic.h"
#include "FuncEst.h"
#include "definitions.h"
#include <iostream>
#include <fstream>
#include <thread>
#include <mutex>

using namespace std;
mutex clog_mtx;
void safe_clog(const string& message) {
    lock_guard<mutex> lock(clog_mtx);
    clog << message << endl;
}

int distToTwo(int hour) {
    int distance = std::abs(hour - 2);
    if (distance > 12) {
        distance = 24 - distance;
    }
    return distance;
}
std::vector<std::vector<double>> road_vectors(PATH_NUM);
vector<string> addHeader = {"toNode", "hourFrom2", "greenProb", "old_vel", "vel"};
vector<string> header;
TrafficHandler traffics;

void TaskTurn(){
    ReadTurn("../../train_turn_cnt.txt");
    FitParam();
    Output("Param_out.txt");
}

void TaskData(const string &mode){
    CSVFile rawTraffic("../../"+mode+"_traffic_data.csv");
    safe_clog(mode+" Read Finish");
    CSVFile dataVel(header);
    int cnt=0;
    double lastVel = 0;
    for(int i=1;i<rawTraffic.size();++i){
        cnt+=1;
        auto &line=rawTraffic[i], &last=rawTraffic[i-1];
        if(last["traj_id"]!=line["traj_id"]){
            lastVel = 0;
            continue;
        }
        int roadID = last["original_path_id"], toID = last["transition_path_id"];
        long long elapsed;
        double vel;
        if(last["original_path_id"]!=line["original_path_id"]) {
            elapsed = last["elapsed"];
            if(elapsed==0)continue;
            vel = last["distance"]/elapsed;
        }
        else{
            elapsed = (long long)(line["sec"]-last["sec"])+(long long)(line["hour"]-last["hour"]+24)%24*3600;
            if(elapsed==0)continue;
            vel = (last["distance"]-line["distance"])/elapsed;
        }
        if(vel>40)continue;
        vector<double>newLine;
        dataVel.append(newLine);
        long long timestamp = last["hour"]*3600+last["sec"];
        dataVel.back().push_back(road_vectors[roadID], last["distance"], distToTwo(last["hour"]),
                                 traffics.query(roadID, toID, timestamp), lastVel, vel);
        if(elapsed<3)lastVel = lastVel/2 + vel/2;
        else lastVel = vel;
    }
    dataVel.saveTo("data_vel_"+mode+".csv");
    safe_clog("Write DataVel.csv Finish");
}

int main(){
    std::vector<std::thread> threads;
    TaskTurn();
    //threads.emplace_back(TaskTurn);
    //for(auto &i:threads)i.join();
    return 0;

    traffics.init("../../train_traffic_data.csv");
    string pref = "vec";
    for(int i=1;i<=vec_len;++i)header.push_back(pref+to_string(i));
    header.insert(header.end(),addHeader.begin(),addHeader.end());
    safe_clog("Start to read files");
    {
        ifstream file("../../road_vectors.txt");
        int num_roads;
        file >> num_roads;
        for (int i = 0; i <= num_roads; ++i) {
            int road_id;
            file >> road_id;
            std::vector<double> vector(vec_len);  // Assuming the vector size is 8
            for (int j = 0; j < vec_len; ++j) file >> vector[j];
            road_vectors[road_id]=vector;

        }
        file.close();
    }
//    threads.emplace_back(TaskData, "train");
//    threads.emplace_back(TaskData, "valid");

    for(auto &i:threads)i.join();
    return 0;
}