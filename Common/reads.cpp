#include "structs.h"
#include "definitions.h"
#include "reads.h"
#include "maths.h"
#include <iostream>
#include <vector>
#include <unordered_map>
#include <string>
#include <fstream>
#include <sstream>
#include <cmath>

using namespace std;
void ReadRoadNet(const std::string& edgeFN, const std::string& typeFN, G&g, Road roads[],
                 GridType&inGrid){
    cout<<"Reading RoadNet..."<<endl;
    ifstream edgeFile(edgeFN);
    string line;
    while(std::getline(edgeFile, line)){
        std::istringstream iss(line);
        int id, from, to, num;
        iss >> id >> from >> to >> num;
        if(num==1){
            double lat,lon;
            iss>>lat>>lon;
            continue;
        }
        g.connect(from,id);
        roads[id]=Road{-1,from,to};
        PointLL last;
        for(int j=0;j<num;++j){
            double lat, lon;
            iss>>lat>>lon;
            PointLL now(lat, lon);
            if(j) {
                PointLL mid = (last+now)/2; // It's just an estimate. The margin of error is very small.
                inGrid[int(mid.lat / GRIDSIZE)][int(mid.lon / GRIDSIZE)].push_back({id, j - 1});
                roads[id].seg.push_back(Segments{Line(last, now), 0, 0});

                if(j>1){
                    Vector vFrom = latLonToXY(roads[id].seg[roads[id].seg.size()-2].line.startLL,last)-Point{0,0};
                    Vector vTo = latLonToXY(now,last)-Point{0,0};
                    float turn = M_PI-Angle(vFrom,vTo);
                    roads[id].angle.push_back(roads[id].angle.back()+turn);
                }
                else roads[id].angle.push_back(0);
            }
            last = now;
        }
        auto &lines = roads[id].seg;
        lines[0].sumPrev=lines[0].line.len;
        lines[lines.size()-1].sumAfter=lines[lines.size()-1].line.len;
        for(int x=1;x<lines.size();++x)lines[x].sumPrev=lines[x-1].sumPrev+lines[x].line.len;
        for(int x=int(lines.size())-2;x>=0;--x)lines[x].sumAfter=lines[x+1].sumAfter+lines[x].line.len;
    }
    edgeFile.close();

    std::ifstream typeFile(typeFN);
    while(std::getline(typeFile, line)){
        std::istringstream iss(line);
        int id, type;
        string typeName;
        iss >> id >> typeName >> type;
        roads[id].level=type;
    }
}

void ReadTraces(const std::string& traceFN, int&m, std::vector<Trace>traces[], bool fourPerRow, bool fixedInterval){
    cout<<"Reading Traces..."<<endl;
    ifstream tr(traceFN);
    long long repeat=1;
    for(m=0;!tr.eof();++m){
        if(m&&m%8192==0)clog<<"\rRead "<<m/1024<<"K trajectories";
        bool ok=true;
        while(true){
            long long stamp, roadID;
            double lat, lon;
            tr>>stamp;
            if(stamp<1){
                if(repeat == stamp){
                    --m;
                    break;
                }
                repeat = stamp;
                break;
            }
            tr>>lat>>lon;
            if(fourPerRow)tr>>roadID;
            if(fixedInterval&&!traces[m].empty()&&abs(stamp-traces[m].back().timestamp)!=RECOVER_INTERVAL){
                cerr<<"In trace "<<m<<", interval is not "<<RECOVER_INTERVAL<<", ignore"<<endl;
                ok=false;
            }
            if(ok)traces[m].push_back({{lat,lon},stamp});
        }
        if(!ok)--m;
    }
    clog<<'\n';
}