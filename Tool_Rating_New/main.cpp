#include "structs.h"
#include "definitions.h"
#include "reads.h"
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
void LCS(const vector<int> &a, const vector<int> &b, vector<int> &result){
    int **dp = new int*[a.size()];
    int **prv = new int*[a.size()];
    for(int i=1;i<a.size();++i){
        dp[i] = new int[b.size()];
        prv[i] = new int[b.size()];
        for(int j=1;j<b.size();++j){
            if(dp[i-1][j]>dp[i][j-1]){
                dp[i][j] = dp[i-1][j];
                prv[i][j] = (i-1)*b.size()+j;
            }
            else{
                dp[i][j] = dp[i][j-1];
                prv[i][j] = i*b.size()+j-1;
            }
            if(dp[i-1][j-1]+1>dp[i][j]){
                dp[i][j] = dp[i-1][j-1]+1;
                prv[i][j] = (i-1)*b.size()+j-1;
            }
        }
    }
    for(int i=1;i<a.size();++i){
        delete[] dp[i];
        delete[] prv[i];
    }
    delete[] dp;
    delete[] prv;
}

int main(){
    ios::sync_with_stdio(false);
    int m;
    ReadRoadNet(EDGEFILE,TYPEFILE,g,roads,inGrid);
    ReadTracesWithRoad("../../test_output.txt", m, traceOut);
    ReadTracesWithRoad("cmake-build-debug/my_output.txt", m, traceSTD);
    ifstream fullOUT("cmake-build-debug/Full_Matched_Dij.txt");
    ifstream fullSTD("cmake-build-debug/Full_Matched_STD.txt");
    int m1,m2;
    fullOUT >> m1; fullSTD >> m2;
    assert(m==m1 && m==m2);
    for(int i=0;i<m;++i){
        int acc=0;
        const auto & trOut = traceOut[i], trSTD = traceSTD[i];
        assert(trOut.size() == trSTD.size());
        double mae = 0, rmse = 0;
        for(int x=0; x < trSTD.size(); ++x){
            acc += (trSTD[x].roadID == trOut[x].roadID);
            double dist = trSTD[x].tr.p.dist(trOut[x].tr.p);
            mae += dist, rmse += dist * dist;
        }
        maeAll += mae / trSTD.size();
        rmseAll += sqrt(rmse) / trSTD.size();
        accuAll += acc / (double)m;

        vector<int>std,out,lcs;
        readIntsFromLine(fullSTD, std);
        readIntsFromLine(fullOUT, out);
        LCS(std,out,lcs);
        double length = 0, stdLen = 0, outLen = 0;
        for(auto x:lcs)length += RoadLen(x);
        for(auto x:std)stdLen += RoadLen(x);
        for(auto x:out)outLen += RoadLen(x);
        double recall = length / stdLen, precision = length / outLen, f1 = recall * precision * 2 / (recall + precision);
        recAll += recall, precAll += precision, f1All += f1;
    }
    cout<<fixed<<setprecision(4);
    cout<<"Road length Recall = "<<recAll/m<<'\n';
    cout<<"Road length Precision = "<<precAll/m<<'\n';
    cout<<"Road length F1 Score = "<<f1All/m<<'\n';
    cout<<"Accuracy = "<<accuAll/m<<'\n';
    cout<<"Straight MAE = "<<maeAll/m<<'\n';
    cout<<"Straight RMSE = "<<rmseAll/m<<'\n';
    cout<<"Road Net MAE = "<<rn_maeAll/m<<'\n';
    cout<<"Road Net RMSE = "<<rn_rmseAll/m<<'\n';
}