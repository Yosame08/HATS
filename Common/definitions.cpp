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

mutex clog_mtx, cout_mtx, cerr_mtx;
void safe_clog(const string& message){
    lock_guard<mutex> lock(clog_mtx);
    clog << '\r' << message << endl;
}
void safe_cout(const string& message){
    lock_guard<mutex> lock(cout_mtx);
    cout << message << endl;
}
void safe_cerr(const string& message){
    lock_guard<mutex> lock(cerr_mtx);
    cerr << message << endl;
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
                float toNodeDist = nowSeg.sumAfter-cross.dist(nowSeg.line.startLL);
                found.push_back({i.roadID, toNodeDist<0?0:toNodeDist, PtMatchProb(dist)});
            }
        }
    }
}

int FindSeg(int roadID, double toNodeDist){
    auto seg=roads[roadID].seg;
    if(toNodeDist>=seg[0].sumAfter||seg.size()==1)return 0;
    int l=0,r=seg.size()-1;
    while(l<r){
        int mid=(l+r)>>1;
        if(seg[mid].sumAfter>=toNodeDist)l=mid+1;
        else r=mid;
    }
    return l-1;
}

float FindAngle(int roadID, double toNodeDist){
    return roads[roadID].angle[FindSeg(roadID,toNodeDist)];
}

PointLL FindLatLon(int roadID, float toNodeDist){
    const auto &seg=roads[roadID].seg[FindSeg(roadID,toNodeDist)];
    auto dir = seg.line.endLL - seg.line.startLL;
    float ratio = (seg.sumAfter - toNodeDist) / seg.line.len;
    return seg.line.startLL + dir * ratio;
}