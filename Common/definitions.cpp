#include "definitions.h"
#include "structs.h"
#include "maths.h"
#include <cmath>
#include <iostream>
#include <queue>
using namespace std;

extern Road roads[];
extern GridType inGrid;

double PtMatchProb(double dist){
    static const double coefficient = 1 / (SIGZ * sqrt(M_2_PI));
    double param = dist / SIGZ;
    return exp(-(param * param * 0.5)) * coefficient;
}

void FindRoad(int dFrom, int dTo, const PointLL &p, vector<Candidate>&found){
    unordered_map<int,priority_queue<pair<double,float>>> tmp;
    int gridX= int(p.lat / GRIDSIZE), gridY= int(p.lon / GRIDSIZE);
    bool ok = false;
    for(int lim = dFrom; lim <= dTo && !ok; lim += dFrom){
        for(int lat = gridX - 20; lat <= gridX + 20; ++lat)for(int lon = gridY - 25; lon <= gridY + 25; ++lon){
            if((!inGrid.count(lat)) || (!inGrid.at(lat).count(lon)))continue;
            for(auto &i:inGrid.at(lat).at(lon)){
                PointLL cross;
                const Segments &nowSeg = roads[i.roadID].seg[i.segID];
                double dist = DistPointSeg(nowSeg.line, p, cross);
                if(dist>lim)continue;
                float toNodeDist = nowSeg.sumAfter-cross.dist(nowSeg.line.startLL);
                tmp[i.roadID].push({PtMatchProb(dist),toNodeDist<0?0:toNodeDist});
                ok = true;
            }
        }
    }
    for(auto &f:tmp){
        double high=f.second.top().first;
        while(!f.second.empty()){
            auto &t=f.second.top();
            if(t.first/high>=EPS)found.push_back({f.first, t.second, t.first});
            else break;
            f.second.pop();
        }
    }
}

int FindSeg(int roadID, double toNodeDist){
    auto seg=roads[roadID].seg;
    if(toNodeDist>=seg[0].sumAfter||seg.size()==1)return 0;
    int l=0,r=int(seg.size()-1);
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