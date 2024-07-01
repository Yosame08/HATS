#include "structs.h"
#include "graph.h"
#include "geography.h"
#include "safeIO.h"
#include <iostream>
#include <fstream>
#include <cassert>
#include <cmath>
#include <iomanip>
#include <cstring>
#include <atomic>
using namespace std;

atomic<int>failed;
struct GPSrd{
    PtEarth p;
    long long timestamp;
    int roadID;
};
using TraceRd = std::vector<GPSrd>;
using TracesRd = std::vector<TraceRd>;

Graph g;
TracesRd standard, user;

double accuAll, maeAll, rmseAll, rn_maeAll, rn_rmseAll, recAll, precAll, f1All;

int readIntsFromLine(std::ifstream& stream, std::vector<int>& numbers) {
    std::string line;
    std::getline(stream, line);
    std::istringstream iss(line);
    int num, cnt=0, last = -1;
    while (iss >> num) {
        if(last==num)continue;
        numbers.push_back(num);
        last = num;
        ++cnt;
    }
    return cnt;
}

/// Solve the largest common sequence of a and b
/// @param result the LCS of a and b
/// @param a array begins with 0
/// @param b array begins with 0
double LCS(const vector<int> &a, const vector<int> &b, vector<int> &result){
    int m = a.size(), n = b.size();
    vector<vector<double>> dp(m + 1, vector<double>(n + 1, 0));

    for (int i = 1; i <= m; ++i) {
        for (int j = 1; j <= n; ++j) {
            if(a[i-1] == b[j-1]) dp[i][j] = (dp[i-1][j-1]+g.lengthOf(a[i-1]));
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

double DistRN(int fromID, int toID, double toNodeA, double fromNodeB){
    if(fromID == toID){
        double len2 = g.lengthOf(fromID) - fromNodeB;
        if(toNodeA>len2)return toNodeA-len2;
        else return len2-toNodeA;
    }
    BitInt vis;
    priority_queue<pair<double,int>>q;
    q.emplace(0,fromID);
    while(!q.empty()){
        int x=q.top().second;
        double len=-q.top().first;
        q.pop();
        if(vis.chk(x))continue;
        vis.set(x);
        for(auto y:g.node[g.getRoad(x).to]){
            if(y!=toID && vis.chk(y))continue;
            if(y==toID)return len + toNodeA + fromNodeB;
            else{
                double dist = len + g.lengthOf(y);
                q.emplace(-dist,y);
            }
        }
    }
    return 1e38;
}

int ReadTracesWithRoad(const std::string &traceFile, TracesRd &traces) {
    safeCOUT("Reading traces from " + traceFile);
    ifstream tr(traceFile);
    if(!tr)throw runtime_error("Cannot open file " + traceFile);
    long long repeat = 1;
    traces.reserve(TraceNumber);
    int m;
    for (m = 0; !tr.eof(); ++m) {
        traces.emplace_back();
        if(traces.size()>10000000)throw runtime_error("Error in reading traces, maybe trouble encountered when reading test input");
        if (!(m & 1023)) safeCOUTRaw("\rRead " + to_string(m >> 10) + "K trajectories");
        bool ok = true;
        while (true) {
            long long stamp;
            int roadID;
            double lat, lon;
            tr >> stamp;
            if (stamp < 1) {
                if (repeat == stamp) {
                    --m;
                    break;
                }
                repeat = stamp;
                break;
            }
            tr >> lat >> lon >> roadID;
            if (!traces[m].empty() && abs(stamp - traces[m].back().timestamp) != RecoverInterval) {
                safeCERR("In trace " + to_string(m) + ", interval is not " + to_string(RecoverInterval) + ", ignored");
                std::this_thread::sleep_for(std::chrono::milliseconds (300));
                ok = false;
            }
            if (ok) traces[m].push_back({{lat, lon}, stamp, roadID});
        }
        if (!ok) --m;
    }
    while((!traces.empty())&&traces.back().empty())traces.pop_back();
    return m;
}

int main(int argc, char* argv[]){
    ios::sync_with_stdio(false);
    int m;
    g.loadEdgeOSM(EdgeFile);
    g.loadTypeOSM(WayTypeFile);

    string recovFN = "recovery"; // default value
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-fn") == 0 && i + 1 < argc) {
            recovFN = argv[i + 1];
            ++i; // skip next argument
        }
        else cout << "redundant argument: " << argv[i] << endl;
    }

    m = ReadTracesWithRoad("../../Dataset/test_output.txt", standard);
    ifstream fullSTD("../../Dataset/test_full_output.txt");
    ReadTracesWithRoad("../../ResultLog/"+recovFN+"_recovery.txt", user);
    ifstream fullOUT("../../ResultLog/"+recovFN+"_full.txt");
    int m1,m2;
    fullOUT >> m1, fullOUT.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    fullSTD >> m2, fullSTD.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    if(m1!=m || m2!=m)cout<<"Warning: "<<"m1="<<m1<<", m2="<<m2<<", m="<<m<<endl;
    assert(m==m1 && m==m2);
    cout<<"Rating..."<<endl;
    for(int i=0;i<m;++i){
        if(i%1024==0)cout<<"\rRated "<<(i>>10)<<"K trajectories"<<flush;
        int acc=0;
        const auto & trOut = user[i], trSTD = standard[i];
        if(trOut.size() != trSTD.size()){
            cout<<i<<" Error: trOut.size() != trSTD.size()"<<endl;
            return 0;
        }
        assert(trOut.size() != 0);
        double mae = 0, rmse = 0, rnmae = 0, rnrmse = 0;
        int skipped = 0;
        for(int x=0; x < trSTD.size(); ++x){
            acc += (trSTD[x].roadID == trOut[x].roadID);
            double dist = trSTD[x].p.dist(trOut[x].p);
            if(dist>5000)cout << x << " Warning: too large distance: " << dist << endl;
            mae += dist, rmse += dist * dist;
            double minSTD=1e38, minOut=1e38, toNodeSTD, toNodeOut;
            for(const auto &y: g.getRoad(trSTD[x].roadID).seg){
                PtEarth out;
                double error = DistPointSeg(y.line, trSTD[x].p, out);
                if(error<minSTD) minSTD = error, toNodeSTD = y.sumAfter - out.dist(y.line.startLL);
            }
            for(const auto &y: g.getRoad(trOut[x].roadID).seg){
                PtEarth out;
                double error = DistPointSeg(y.line, trOut[x].p, out);
                if(error<minOut) minOut = error, toNodeOut = y.sumAfter - out.dist(y.line.startLL);
            }
            assert(minSTD != 1e38 && minOut != 1e38);
            double minDist = min(DistRN(trSTD[x].roadID, trOut[x].roadID, toNodeSTD, g.lengthOf(trOut[x].roadID)-toNodeOut),
                                DistRN(trOut[x].roadID, trSTD[x].roadID, toNodeOut, g.lengthOf(trSTD[x].roadID)-toNodeSTD));
            if(minDist > 1e37) ++skipped;
            else rnmae += minDist, rnrmse += minDist * minDist;
        }
        maeAll += mae / trSTD.size();
        rmseAll += sqrt(rmse / trSTD.size());
        rn_maeAll += rnmae / (trSTD.size() - skipped);
        rn_rmseAll += sqrt(rnrmse / (trSTD.size() - skipped));
        accuAll += acc / (double)trSTD.size();

        vector<int>std,out,lcs;
        // with deduplication
        readIntsFromLine(fullSTD, std);
        readIntsFromLine(fullOUT, out);
        double length = LCS(std,out,lcs), stdLen = 0, outLen = 0;
        for(auto x:std)stdLen += g.lengthOf(x);
        for(auto x:out)outLen += g.lengthOf(x);
        double recall = length / stdLen, precision = length / outLen;
        if(recall == 0 || precision == 0){
            cout<<"In "<<i<<": recall = "<<recall<<"; precision = "<<precision<<endl;
        }
        double f1;
        if (recall == 0 && precision == 0) f1 = 0;
        else f1 = recall * precision * 2 / (recall + precision);
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
