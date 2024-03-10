#include "definitions.h"
#include "structs.h"
#include "maths.h"
#include <cmath>
#include <mutex>
#include <cassert>
#include <iostream>
using namespace std;

extern Road roads[];
extern GridType inGrid;

mutex clog_mtx;
void safe_clog(const string& message){
    lock_guard<mutex> lock(clog_mtx);
    clog << '\r' << message << endl;
}

double PtMatchProb(double dist){
    static const double coefficient = 1 / (SIGZ * sqrt(M_2_PI));
    double param = dist / SIGZ;
    return exp(-(param * param * 0.5)) * coefficient;
}

void FindRoad(int dFrom, int dTo, const PointLL &p, vector<Candidate>&found){
    int gridX= int(p.lat / GRIDSIZE), gridY= int(p.lon / GRIDSIZE);
    for(int lim = dFrom; found.empty() && lim <= dTo; lim+=dFrom){
        for(int lat = gridX - 15; lat <= gridX + 15; ++lat)for(int lon = gridY - 25; lon <= gridY + 25; ++lon){
            if((!inGrid.count(lat)) || (!inGrid.at(lat).count(lon)))continue;
            for(auto &i:inGrid.at(lat).at(lon)){
                PointLL cross;
                const Segments &nowSeg = roads[i.roadID].seg[i.segID];
                double dist = DistPointSeg(nowSeg.line, p, cross);
                if(dist>lim)continue;
                found.push_back({i.roadID,float(nowSeg.sumAfter-cross.dist(nowSeg.line.startLL)), PtMatchProb(dist)});
            }
        }
    }
}

float FindAngle(int roadID, double toNodeDist){
    for(int i=roads[roadID].seg.size()-1; i>=0; --i){
        if(roads[roadID].seg[i].sumAfter>=toNodeDist)return roads[roadID].angle[i];
    }
    assert(false);
}