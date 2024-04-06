#ifndef MYDEFINES_H
#define MYDEFINES_H

#define EDGEFILE "../Map/edgeOSM.txt"
#define TYPEFILE "../Map/wayTypeOSM.txt"
#define TRACEFILE "../test_sampled.txt"
#define ROADVECTOR "../Intermediate/road_vectors.txt"
#define GRIDSIZE 0.001 // 0.001 latitude = 111.195m
#define PATH_NUM 65536
#define TIMEZONE 0
#define SIGZ 4.07 // Related to the confidence of the GPS location point. The more reliable the location point is, the smaller σ_z should be.
#define BETA 2 // Affects the confidence calculated by the size of the difference between Great Circle Distance and Route Distance
#define EPS 1e-5
#define RECOVER_INTERVAL 15
#define vec_len 13

// Macros for [FuncEst.h/cpp] to fit parameters
#define granularity 2
#define PARAMTURN "../Intermediate/ParamTurn.txt"
#define PARAMLENPOS "../Intermediate/ParamLenPos.txt"
#define PARAMLENNEG "../Intermediate/ParamLenNeg.txt"
#define mxMissing 25 // maximum number of consecutive missing records in the trajectory
#define minProb 1e-150 // minProb^2 doesn't exceed double

#include <vector>
#include <unordered_map>
#include <string>

struct GridInfo;
struct PointLL;
struct Road;
struct Candidate;
using GridType = std::unordered_map<int,std::unordered_map<int,std::vector<GridInfo>>>;

double PtMatchProb(double dist);
void FindRoad(int dFrom, int dTo, const PointLL &p, std::vector<Candidate>&found);
void FindRoadMulti(int dFrom, int dTo, const PointLL &p, std::vector<Candidate>&found);
/*
 * Find out the total degrees of turning from the starting point of the segment
 */
float FindAngle(int roadID, double toNodeDist);
PointLL FindLatLon(int roadID, float toNodeDist);
float RoadLen(int roadID);
double GetTurnAngle(int fromID, int toID);

#endif //MYDEFINES_H
