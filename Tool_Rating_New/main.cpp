#include "structs.h"
#include "definitions.h"
#include "reads.h"
#include "maths.h"
#include <iostream>
#include <fstream>
#include <cassert>
#include <cmath>
#include <iomanip>
using namespace std;

G g;
Road roads[PATH_NUM];
vector<TraceRD>traceOut[524288], traceSTD[524288];
GridType inGrid;

double accuAll, maeAll, rmseAll, rn_maeAll, rn_rmseAll, recAll, precAll, f1All;

/// Solve the largest common sequence of a and b
/// @param result the LCS of a and b
/// @param a array begins with 0
/// @param b array begins with 0
float LCS(const vector<int> &a, const vector<int> &b, vector<int> &result){
    int m = a.size(), n = b.size();
    vector<vector<float>> dp(m + 1, vector<float>(n + 1, 0));

    for (int i = 1; i <= m; ++i) {
        for (int j = 1; j <= n; ++j) {
            if(a[i-1] == b[j-1]) dp[i][j] = (dp[i-1][j-1]+RoadLen(a[i-1]));
            else dp[i][j] = max(dp[i-1][j], dp[i][j-1]);
        }
    }

    int i=m, j=n;
    while (i>0 && j>0) {
        if (a[i-1] == b[j-1]) {
            result.push_back(a[i-1]);
            --i, --j;
        }
        else if (dp[i-1][j] > dp[i][j-1]) --i;
        else --j;
    }
    std::reverse(result.begin(), result.end());
    return dp[m][n];
}

float DistRN(int fromID, int toID, float toNodeA, float fromNodeB){
    if(fromID == toID){
        float len2 = RoadLen(fromID) - fromNodeB;
        if(toNodeA>len2)return toNodeA-len2;
        else return len2-toNodeA;
    }
    BitInt vis;
    priority_queue<pair<float,int>>q;
    q.emplace(0,fromID);
    while(!q.empty()){
        int x=q.top().second;
        float len=-q.top().first;
        q.pop();
        if(vis.chk(x))continue;
        vis.set(x);
        for(auto y:g.node[roads[x].to]){
            if(y!=toID && vis.chk(y))continue;
            if(y==toID)return len + toNodeA + fromNodeB;
            else{
                float dist = len + RoadLen(y);
                q.emplace(-dist,y);
            }
        }
    }
    return 1e38;
}

int main(){
    ios::sync_with_stdio(false);
    int m;
    ReadRoadNet(EDGEFILE,TYPEFILE,g,roads,inGrid);
    ReadTracesWithRoad("../../test_output.txt", m, traceSTD);
    ifstream fullSTD("Full_Matched_STD.txt");
    ReadTracesWithRoad("../../RecoveryHistory/4.2minus300.txt", m, traceOut);
    ifstream fullOUT("../../RecoveryHistory/4.2minus300Full.txt");
    int m1,m2;
    fullOUT >> m1, fullOUT.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    fullSTD >> m2, fullSTD.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    assert(m==m1 && m==m2);
    cout<<"Rating..."<<endl;
    for(int i=0;i<m;++i){
        if(i%1024==0)cout<<"\rRated "<<(i>>10)<<"K trajectories"<<flush;
        int acc=0;
        const auto & trOut = traceOut[i], trSTD = traceSTD[i];
        assert(trOut.size() == trSTD.size());
        double mae = 0, rmse = 0, rnmae = 0, rnrmse = 0;
        for(int x=0; x < trSTD.size(); ++x){
            acc += (trSTD[x].roadID == trOut[x].roadID);
            double dist = trSTD[x].tr.p.dist(trOut[x].tr.p);
            mae += dist, rmse += dist * dist;
            double minSTD=1e38, minOut=1e38;
            float toNodeSTD, toNodeOut;
            for(const auto &y: roads[trSTD[x].roadID].seg){
                PointLL out;
                double error = DistPointSeg(y.line, trSTD[x].tr.p, out);
                if(error<minSTD) minSTD = error, toNodeSTD = y.sumAfter - out.dist(y.line.startLL);;
            }
            for(const auto &y: roads[trOut[x].roadID].seg){
                PointLL out;
                double error = DistPointSeg(y.line, trOut[x].tr.p, out);
                if(error<minOut) minOut = error, toNodeOut = y.sumAfter - out.dist(y.line.startLL);;
            }
            assert(minSTD != 1e38 && minOut != 1e38);
            float minDist = min(DistRN(trSTD[x].roadID, trOut[x].roadID, toNodeSTD, RoadLen(trOut[x].roadID)-toNodeOut),
                                DistRN(trOut[x].roadID, trSTD[x].roadID, toNodeOut, RoadLen(trSTD[x].roadID)-toNodeSTD));
            assert(minDist != 1e38);
            rnmae += minDist, rnrmse += minDist * minDist;
        }
        maeAll += mae / trSTD.size();
        rmseAll += sqrt(rmse / trSTD.size());
        rn_maeAll += rnmae / trSTD.size();
        rn_rmseAll += sqrt(rnrmse / trSTD.size());
        accuAll += acc / (double)trSTD.size();

        vector<int>std,out,lcs;
        // with deduplication
        readIntsFromLine(fullSTD, std);
        readIntsFromLine(fullOUT, out);
        float length = LCS(std,out,lcs), stdLen = 0, outLen = 0;
        for(auto x:std)stdLen += RoadLen(x);
        for(auto x:out)outLen += RoadLen(x);
        double recall = length / stdLen, precision = length / outLen, f1 = recall * precision * 2 / (recall + precision);
        recAll += recall, precAll += precision, f1All += f1;
    }
    cout<<fixed<<setprecision(4)<<'\n';
    cout<<"************ Result ************\n";
    cout<<"Road length Recall = "<<recAll/m<<'\n';
    cout<<"Road length Precision = "<<precAll/m<<'\n';
    cout<<"Road length F1 Score = "<<f1All/m<<'\n';
    cout<<"Accuracy = "<<accuAll/m<<'\n';
    cout<<"Straight MAE = "<<maeAll/m<<'\n';
    cout<<"Straight RMSE = "<<rmseAll/m<<'\n';
    cout<<"Road Net MAE = "<<rn_maeAll/m<<'\n';
    cout<<"Road Net RMSE = "<<rn_rmseAll/m<<'\n';
    return 0;
}