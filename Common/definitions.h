#ifndef MYDEFINES_H
#define MYDEFINES_H

#define EDGEFILE "../../edgeOSM_Porto.txt"
#define TYPEFILE "../../wayTypeOSM_Porto.txt"
#define TRACEFILE "../../test_sampled.txt"
#define PARAMTURN "../../ParamTurn.txt"
#define PARAMLEN "../../ParamLen.txt"
#define GRIDSIZE 0.001 // 0.001 latitude = 111.195m
#define PATH_NUM 65536
#define TIMEZONE 0
#define SIGZ 4.07    // Related to the confidence of the GPS location point. The more reliable the location point is, the smaller σ_z should be.
#define BETA 2.5 // Affects the confidence calculated by the size of the difference between Great Circle Distance and Route Distance
#define EPS 1e-5
#define RECOVER_INTERVAL 15
#define vec_len 12
#define granular_turn 1 // for calculating the probability of turning
#define granular_len 1 // for calculating the probability of turning
#include <vector>
#include <unordered_map>
#include <string>

struct GridInfo;
struct PointLL;
struct Road;
struct Candidate;
using GridType = std::unordered_map<int,std::unordered_map<int,std::vector<GridInfo>>>;

void safe_clog(const std::string& message);
double PtMatchProb(double dist);
void FindRoad(int dFrom, int dTo, const PointLL &p, std::vector<Candidate>&found);

/*
 * Find out the total degrees of turning from the starting point of the segment
 */
float FindAngle(int roadID, double toNodeDist);
PointLL FindLatLon(int roadID, float toNodeDist);

#endif //MYDEFINES_H
