//
// Created by yosame on 24-6-15.
//

#ifndef INC_4_SOLVE_GRAPH_H
#define INC_4_SOLVE_GRAPH_H

#include "road.h"
#include <string>
#include <vector>

class Graph {
public:
    void loadEdgeOSM(const std::string &filename);
    void loadTypeOSM(const std::string &filename);
    double lengthOf(int roadID) const;
    int levelOf(int roadID) const;
    PtEarth endOf(int roadID) const;
    int roadTo(int roadID) const;
    void Point2RoadUnique(int rStart, const PtEarth &p, std::vector<Candidate> &found);
    int SegmentID(int roadID, double toNodeDist) const;
    double TurnedAngle(int roadID, double toNodeDist) const;
    double DiscontinuousAngle(int fromID, int toID) const;
    PtEarth Road2Point(int roadID, double toNodeDist) const;
    std::vector<int> node[NodeNumber];
    const Road &getRoad(int roadID) const;
    double GetTurnAngle(int fromID, int toID);
private:
    static const int rangeLim = 1000;// in meter
    Road roads[RoadNumber];
    RoadGrid inGrid;
    void connect(int nodeID, int roadID);
};


#endif//INC_4_SOLVE_GRAPH_H
