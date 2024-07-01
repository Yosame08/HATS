//
// Created by yosame on 24-6-15.
//

#include "graph.h"
#include "definitions.h"
#include "geography.h"
#include "road.h"
#include "safeIO.h"
#include <cmath>
#include <fstream>
#include <sstream>
using namespace std;

void Graph::loadEdgeOSM(const std::string &filename) {
    safeCOUT("Loading roads from " + filename);
    ifstream edgeFile(filename);
    string line;
    while (std::getline(edgeFile, line)) {
        std::istringstream iss(line);
        int id, from, to, num;
        iss >> id >> from >> to >> num;
        roads[id] = Road{-1, from, to};
        if (num == 1) {// ignore the road with only one point
            double lat, lon;
            iss >> lat >> lon;
            continue;
        }
        PtEarth last{-1024, -1024};
        for (int i = 0; i < num; ++i) {
            double lat, lon;
            iss >> lat >> lon;
            PtEarth now(lat, lon);
            if (now == last) continue;
            if (!(last == PtEarth{-1024, -1024})) {
                PtEarth mid = (last + now) / 2;// It's just an estimate. The margin of error is very small.
                inGrid[{static_cast<int>(mid.lat / GridSize), static_cast<int>(mid.lon / GridSize)}].push_back({id, i - 1});
                if (!roads[id].seg.empty()) {
                    Vector vFrom = latLonToXY(roads[id].seg.back().line.startLL, last) - Point{0, 0};
                    Vector vTo = latLonToXY(now, last) - Point{0, 0};
                    double turn = M_PI - Angle(vFrom, vTo);
                    roads[id].angle.push_back(roads[id].angle.back() + turn);
                } else
                    roads[id].angle.push_back(0);
                roads[id].seg.push_back(Segment{Line(last, now), 0, 0});
            }
            last = now;
        }
        auto &lines = roads[id].seg;
        if (lines.empty()) continue;
        // There might be two points with the same latitude and longitude
        connect(from, id);
        lines[0].sumPrev = lines[0].line.len;
        lines[lines.size() - 1].sumAfter = lines[lines.size() - 1].line.len;
        for (int x = 1; x < lines.size(); ++x) lines[x].sumPrev = lines[x - 1].sumPrev + lines[x].line.len;
        for (int x = int(lines.size()) - 2; x >= 0; --x) lines[x].sumAfter = lines[x + 1].sumAfter + lines[x].line.len;
    }
    edgeFile.close();
}

void Graph::loadTypeOSM(const string &filename) {
    safeCOUT("Loading types of roads from " + filename);
    ifstream edgeFile(filename);
    std::ifstream typeFile(filename);
    string line;
    while (std::getline(typeFile, line)) {
        std::istringstream iss(line);
        int id, type;
        string typeName;
        iss >> id >> typeName >> type;
        roads[id].level = type - 1;
    }
}

void Graph::connect(int nodeID, int roadID) {
    node[nodeID].push_back(roadID);
}

double Graph::lengthOf(int roadID) const {
    return roads[roadID].length();
}

int Graph::levelOf(int roadID) const {
    return roads[roadID].level;
}

PtEarth Graph::endOf(int roadID) const {
    return roads[roadID].seg.back().line.endLL;
}

int Graph::roadTo(int roadID) const {
    return roads[roadID].to;
}

void Graph::Point2RoadUnique(int rStart, const PtEarth &p, vector<Candidate> &found) {
    unordered_map<int, pair<double, double>> tmp;
    int gridX = static_cast<int>(p.lat / GridSize);
    int gridY = static_cast<int>(p.lon / GridSize);
    for (int lim = rStart; lim <= rangeLim && tmp.size() < 2; lim += rStart) {
        pair<int, int> now;
        for (now.first = gridX - 20; now.first <= gridX + 20; ++now.first) {
            for (now.second = gridY - 25; now.second <= gridY + 25; ++now.second) {
                if ((!inGrid.count(now))) continue;
                for (auto &i: inGrid.at(now)) {
                    PtEarth cross;
                    const Segment &nowSeg = roads[i.roadID].seg[i.segID];
                    double dist = DistPointSeg(nowSeg.line, p, cross);
                    if (dist > lim) continue;
                    double toNodeDist = nowSeg.sumAfter - cross.dist(nowSeg.line.startLL);
                    if((!tmp.count(i.roadID)) || dist<tmp.at(i.roadID).first)tmp[i.roadID] = {dist, toNodeDist < 0 ? 0 : toNodeDist};
                }
            }
        }
    }
    for (auto &f: tmp) {
        found.push_back({f.first, f.second.second, f.second.first});
    }
}

int Graph::SegmentID(int roadID, double toNodeDist) const {
    const auto &seg = roads[roadID].seg;
    if (toNodeDist >= seg[0].sumAfter) return 0;
    switch (seg.size()) {
        case 0:
            throw runtime_error("SegmentID: roadID = " + to_string(roadID) + ", seg.size() == 0");
        case 1:
            return 0;
        case 2:
            return toNodeDist < seg[1].sumAfter ? 1 : 0;
        case 3:
            if (toNodeDist < seg[2].sumAfter) return 2;
            if (toNodeDist < seg[1].sumAfter) return 1;
            return 0;
        case 4:
            if (toNodeDist < seg[3].sumAfter) return 3;
            if (toNodeDist < seg[2].sumAfter) return 2;
            if (toNodeDist < seg[1].sumAfter) return 1;
            return 0;
        default:
            int l = 0, r = int(seg.size() - 1);
            while (l < r) {
                int mid = (l + r) >> 1;
                if (seg[mid].sumAfter >= toNodeDist) l = mid + 1;
                else
                    r = mid;
            }
            return l - 1;
    }
}

double Graph::TurnedAngle(int roadID, double toNodeDist) const {
    return roads[roadID].angle[SegmentID(roadID, toNodeDist)];
}

double Graph::GetTurnAngle(int fromID, int toID){
    const PtEarth &cross = roads[toID].seg.front().line.startLL;
    Vector vFrom = latLonToXY(roads[fromID].seg.back().line.startLL,cross)-Point{0,0};
    Vector vTo = latLonToXY(roads[toID].seg.front().line.endLL,cross)-Point{0,0};
    return M_PI - Angle(vFrom,vTo);
}

double Graph::DiscontinuousAngle(int fromID, int toID) const {
    const PtEarth &cross = roads[toID].seg.front().line.startLL;
    Vector vFrom = latLonToXY(roads[fromID].seg.back().line.startLL, cross) -
                   latLonToXY(roads[fromID].seg.back().line.endLL, cross);
    Vector vTo = latLonToXY(roads[toID].seg.front().line.endLL, cross) - Point{0, 0};
    return M_PI - Angle(vFrom, vTo);
}

PtEarth Graph::Road2Point(int roadID, double toNodeDist) const {
    const auto &seg = roads[roadID].seg[SegmentID(roadID, toNodeDist)];
    auto dir = seg.line.endLL - seg.line.startLL;
    double ratio = (seg.sumAfter - toNodeDist) / seg.line.len;
    return seg.line.startLL + dir * ratio;
}

const Road &Graph::getRoad(int roadID) const {
    return roads[roadID];
}
