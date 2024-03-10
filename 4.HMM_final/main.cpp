#include "definitions.h"
#include "structs.h"
#include "reads.h"
#include "maths.h"
#include "predict.h"
#include "FuncEst.h"
#include <fstream>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <stack>
#include <queue>
#include <thread>
#include <atomic>
#include <unordered_set>
using namespace std;

G g;
Road roads[PATH_NUM];
vector<Trace>traces[262144];
ostringstream recovStream[262144], matchStream[262144];
GridType inGrid;
vector<float> road_vectors[PATH_NUM];
void ReadVectors();

float RoadLen(int roadID){
    return roads[roadID].seg.back().sumPrev;
}

double DifDistProb(double dif, unsigned long long span) {
    double ans;
    if(dif<0)ans = exp(-dif / span / 0.7) / 0.7;
    else ans = exp(-dif / span / 1.5) / 1.5;
    return ans==0 ? 1e-300 : ans; // return 0 may cause error
}

double SearchDifDistProb(double dif, unsigned long long span) {
    if(dif<0)return 1;
    double ans = exp(-dif / span / 1.5) / 1.5;
    return ans==0 ? 1e-300 : ans; // return 0 may cause error
}

double TypeChangeProb(int _old, int _new){
    if(_old==-1)return pow(2,_new<=4?_new-4:4-_new);
    if(_new==_old)return 1;
    return 0.5*pow(2,_new<=4?_new-4:4-_new);
}

/// @param dire RAD
/// @brief Convert radians to angles and predict with the fitted function
double AngleProb(double angle, long long time){
    static vector<int> times = LoadParam(PARAMFILE);
    int l,r;
    auto f = lower_bound(times.begin(), times.end(), time);
    if(f==times.end())l=r=(int)times.size()-1; // time > maximum interval
    else if(f==times.begin()&&time<*f)l=r=0; // time < minimum interval
    else if(time==*f)l=r=f-times.begin();
    else{
        r = f-times.begin();
        l = r - 1;
    }
    if(l==r)return Estimate_wrap(angle*180/M_PI/time, l);
    return (Estimate_wrap(angle*180/M_PI/time,l)+Estimate_wrap(angle*180/M_PI/time,r))/2;
}

SearchRes SearchRoad(int fromRoad, int toRoad, float toNodeDistA, float fromNodeDistB, const Trace &lastTr, const Trace &nowTr){
    // seqPath is to restore nodes searched. Every node restore information about current road and which node is previous road
    vector<pair<PathNode,int>>seqPath;
    // Use DP to simplify searching process. Like Dijkstra but use "probability" as key for sort
    bool vis[PATH_NUM+1]{}; priority_queue<QueueInfo>q;
    // Initialize some const Value to prune search tree
    const float greatCircle = lastTr.p.dist(nowTr.p);
    const unsigned long long span = nowTr.timestamp - lastTr.timestamp;
    // Set starting state
    seqPath.push_back({{fromRoad, lastTr.timestamp, toNodeDistA}, -1});
    q.push(QueueInfo{1, (int)seqPath.size()-1, TypeChangeProb(-1,roads[fromRoad].level), toNodeDistA, FindAngle(fromRoad,0)-FindAngle(fromRoad,toNodeDistA)});
    SearchRes result{-1};

    while(!q.empty()){
        const auto top=q.top();q.pop();
        const auto lastPath = seqPath[top.node].first;
        if(vis[lastPath.roadID])continue;
        vis[lastPath.roadID]=true;
        int node = roads[lastPath.roadID].to;
        for(auto to:g.node[node]){
            if(vis[to])continue;
            const PointLL &cross = roads[to].seg.front().line.startLL;
            Vector vTo = latLonToXY(roads[to].seg.front().line.endLL,cross)-Point{0,0};
            Vector vFrom = latLonToXY(roads[seqPath[top.node].first.roadID].seg.back().line.startLL,cross)-Point{0,0};
            float angle = top.angle + M_PI - Angle(vFrom,vTo);
            // double tranProb = top.accuProb / SearchDifDistProb(top.len - greatCircle, span)
            //                   * SearchDifDistProb(totLen - greatCircle, span)
            //                   * TypeChangeProb(roads[lastPath.roadID].level, roads[to].level);
            double tranProb = top.accuProb / SearchDifDistProb(top.len - greatCircle, span)
                                           / AngleProb(top.angle, span)
                                           * TypeChangeProb(roads[lastPath.roadID].level, roads[to].level);
            // 计算转弯概率，忽略长5m以下的道路
            // if(RoadLen(to)>5){
            //     for(int oldnode=top.node;oldnode!=-1;oldnode=seqPath[oldnode].second){
            //         const int roadID = seqPath[oldnode].first.roadID;
            //         if(RoadLen(roadID)>5){
            //             const PointLL &cross = roads[roadID].seg.back().line.endLL;
            //             Vector vFrom = latLonToXY(roads[roadID].seg.back().line.startLL,cross)-Point{0,0};
            //             Vector vTo = latLonToXY(roads[to].seg.front().line.endLL,cross)-Point{0,0};
            //             tranProb *= Transition(M_PI-Angle(vFrom,vTo));
            //             break;
            //         }
            //     }
            // }
            // 找到目的地，计算概率
            if(to==toRoad){
                float allLen = top.len + fromNodeDistB;
                if(allLen >= span * 45)continue;
                angle += FindAngle(toRoad, RoadLen(toRoad)-fromNodeDistB);
                // 如果是终点，没有走到头，修改计算的概率
                double outProb = tranProb * DifDistProb(allLen - greatCircle, span)
                                          * AngleProb(angle, span);
                if(outProb>result.prob){
                    seqPath.push_back({{to, lastPath.timestamp, (float)RoadLen(toRoad)}, top.node});
                    result = {outProb, allLen, angle, (int)seqPath.size()-1};
                }
                continue;
            }
            float totLen = top.len + RoadLen(to);
            angle += FindAngle(to,0);
            tranProb *= SearchDifDistProb(totLen - greatCircle, span) * AngleProb(angle, span);
            // 检查应该入队还是被剪枝
            // 各种剪枝：最多执行span/2步，也就是最快允许每2秒换一条路段
            if(top.level + 1 > span/2)continue;
            // span/4步后，比起点前更靠近目的地（直线）
            if(top.level>span/4 && greatCircle<nowTr.p.dist(roads[to].seg.back().line.endLL))continue;
            // 车速极快
            if(totLen >= span * 45)continue;
            // 通过剪枝，入队
            seqPath.push_back({{to, lastPath.timestamp, (float)RoadLen(to)}, top.node});
            q.push({top.level+1, (int)seqPath.size()-1, tranProb, totLen, angle});
        }
    }
    // 搜索结束
    if(result.prob==-1)return result; // Search Failed
    stack<int>choose;
    for(int x=result.node;x!=-1;x=seqPath[x].second)choose.push(x);
    while(!choose.empty()){
        result.path.push_back(seqPath[choose.top()].first);
        choose.pop();
    }
    result.path.push_back({toRoad, nowTr.timestamp, (float)RoadLen(toRoad)-fromNodeDistB});
    return result;
}

void TurnInterpolation(int fromRoad, int toRoad, double toNodeDistA, double fromNodeDistB, long long lastStamp, long long nowStamp, Path& path){
    const unsigned long long span = nowStamp - lastStamp;
    long long tm = lastStamp;
    auto predict = [&](int roadID, int toID, double toNodeDist, double needPass){
        const int DistEPS=1;
        while(needPass>DistEPS){
            double vel = VelPrediction(roadID, toID, toNodeDist, tm);
            if(vel<0.01)vel=0.01;
            ++tm;
            needPass-=vel;
            toNodeDist-=vel;
        }
    };
    double toNodeDist = toNodeDistA;
    for(int i=1, roadID = fromRoad;i<path.size()-1;++i){
        predict(roadID, path[i].roadID, toNodeDist, toNodeDist);
        path[i].timestamp = tm;
        roadID=path[i].roadID;
        toNodeDist=RoadLen(roadID);
    }
    predict(toRoad, -1, RoadLen(toRoad), fromNodeDistB);
    long long calcSpan = tm-lastStamp;
    double revise = 1/(calcSpan/(double)span);
    for(int i=1;i<path.size()-1;++i)path[i].timestamp = lastStamp+(path[i].timestamp-lastStamp)*revise;
}

void solve(int id, ostringstream &match, ostringstream &recovery){
    auto myAssert = [&](bool condition, const string& cause){
        if(condition)return true;
        recovery<<"Failed\n"<<-id-1<<'\n';
        safe_clog(string("Can't Match point to road at road id ")+ to_string(id) +string(" due to ")+cause);
        return false;
    };
    const auto& traceNow = traces[id];
    // Use Viterbi Alg. to perform matching - "prob" variable in SearchNode as a key value
    vector<Candidate>found;
    vector<SearchNode>search;
    vector<long long>timestamps;
    timestamps.push_back(traceNow[0].timestamp);
    FindRoad(60, 240, traceNow[0].p, found);
    if(!myAssert(!found.empty(), "Can't match point 0 to a road"))return;
    search.reserve(found.size());
    for(auto &x:found)search.push_back({x.prob,x.toNodeDist,x.roadID,-1,0,{{x.roadID,traceNow[0].timestamp,x.toNodeDist}}});
    int oldBegin=0, oldEnd=(int)search.size()-1;
    unordered_set<unsigned long long>noPath;

    for(int i=1;i<traceNow.size();++i){
        for(long long t=timestamps.back()+RECOVER_INTERVAL;t<traceNow[i].timestamp;t+=RECOVER_INTERVAL)timestamps.push_back(t);
        timestamps.push_back(traceNow[i].timestamp);
        found.clear();
        FindRoad(60, 240, traceNow[i].p, found);
        if(!myAssert(!found.empty(), "Can't match point "+ to_string(i)+" to a road"))return;

        for(auto &now:found){
            SearchNode maxNode{0};
            for(int l=oldBegin;l<=oldEnd;++l){
                SearchNode &old = search[l];
                unsigned long long hashed = now.roadID + ((unsigned long long)old.roadID << 32llu);
                if(noPath.count(hashed))continue;
                Path path;
                double traceProb;
                if(old.roadID == now.roadID){
                    double ground = old.toNodeDist - now.toNodeDist;
                    traceProb = DifDistProb(traceNow[i - 1].p.dist(traceNow[i].p) - ground, traceNow[i].timestamp-traceNow[i-1].timestamp);
                    path.push_back({now.roadID,traceNow[i].timestamp,now.toNodeDist});
                    float angle = FindAngle(now.roadID, now.toNodeDist) - FindAngle(old.roadID, old.toNodeDist);
                    traceProb *= AngleProb(angle, traceNow[i].timestamp-traceNow[i-1].timestamp);
                }
                else{
                    SearchRes result = SearchRoad(old.roadID, now.roadID, old.toNodeDist,
                                                  RoadLen(now.roadID) - now.toNodeDist, traceNow[i-1], traceNow[i]);
                    if(result.prob==-1){
                        noPath.insert(hashed);
                        continue;
                    }
                    traceProb = result.prob;
                    path = result.path;
                }
                // now.prob is the difference between GPS point and fullPath point
                double allProb = old.prob * pow(now.prob * traceProb, 1/2.0);
                if(allProb > maxNode.prob)maxNode = {allProb, now.toNodeDist, now.roadID, l, i, path};
            }
            if(maxNode.prob>0)search.push_back(maxNode);
        }
        if(search.size()-1==oldEnd){ // HMM break
            safe_clog(string("HMM break at PATH ")+ to_string(id) + string(" trace ")+ to_string(i));
            int mxPoint=0;double mxProb=-1;
            for (int k = oldBegin; k <= oldEnd; ++k)if(search[k].prob > mxProb)mxProb=search[k].prob, mxPoint=k;
            for(auto &now:found)
                search.push_back({now.prob, now.toNodeDist, now.roadID, mxPoint, i, {{now.roadID,traceNow[i].timestamp,now.toNodeDist}}});
        }
        oldBegin = oldEnd + 1, oldEnd = (int)search.size()-1;
        double maxProb=-1;
        for(int x=oldBegin;x<=oldEnd;++x)if(search[x].prob>maxProb)maxProb=search[x].prob;
        if(maxProb!=0)for(int x=oldBegin;x<=oldEnd;++x)search[x].prob/=maxProb;
    }
    double maxProb=0;
    int outID;
    for(int i=oldBegin;i<=oldEnd;++i)if(search[i].prob>maxProb)maxProb=search[i].prob, outID=i;
    Path matched;
    PathNode interpolation{-1,-1,-1};
    while(outID!=-1){
        auto &node=search[outID], &last=search[node.prev];
        if(!myAssert(!node.path.empty(),"(Bug) Exists a node with no path at trace"+to_string(id)))return;
        if(node.prev!=-1){
            TurnInterpolation(last.roadID,node.roadID,last.toNodeDist, RoadLen(node.roadID)-node.toNodeDist,traceNow[last.pointID].timestamp,traceNow[node.pointID].timestamp,node.path);
        }
        for(int i=(int)node.path.size()-1;i>=0;--i)matched.push_back(node.path[i]);
        outID=node.prev;
    }
    int lastOut=-1,rear=int(matched.size())-1,nextOut=rear,timeID=0;
    for(int p=rear;p>=0;--p){
        auto &now=matched[p];
        if(now.roadID!=lastOut){
            lastOut=now.roadID;
            match<<now.roadID<<' ';
        }
        if(p==nextOut){
            for(int x=p-1;x>=0;--x)if(matched[x].roadID!=now.roadID){
                    nextOut=x;
                    break;
                }
            if(now.roadID==matched[nextOut].roadID)nextOut=-1;
        }
        while(timeID<timestamps.size()&&timestamps[timeID]<=now.timestamp){
            // Every turn is recorded
            int roadID; float toNodeDist;
            if(timestamps[timeID]==now.timestamp){
                roadID=now.roadID;
                toNodeDist=now.toNodeDist;
            }else{
                //interpolation
                auto &last = matched[p+1].timestamp>=interpolation.timestamp?matched[p+1]:interpolation;
                roadID = last.roadID;
                toNodeDist = last.toNodeDist;
                for(long long span=0; toNodeDist>0 && span<timestamps[timeID]-last.timestamp; ++span){
                    float vel = VelPrediction(roadID, p==nextOut?-1:matched[nextOut].roadID, toNodeDist, last.timestamp+span);
                    if(vel<0.01)vel=0.01;
                    toNodeDist -= vel;
                    if(toNodeDist<0){
                        toNodeDist=0;
                        break;
                    }
                }
            }
            PointLL recov{-1024,-1024};
            for(int i=int(roads[roadID].seg.size())-1;i>=0;--i){
                auto &seg=roads[roadID].seg[i];
                if(seg.sumAfter >= toNodeDist - EPS){
                    double per2 = seg.line.len>0 ? (seg.sumAfter - toNodeDist) / seg.line.len : 1;
                    recov=seg.line.startLL+(seg.line.endLL-seg.line.startLL)*per2;
                    break;
                }
            }
            interpolation = PathNode{roadID,timestamps[timeID],toNodeDist};
            if(!myAssert(!(recov==PointLL(-1024,-1024)), "(Bug) Bad value: toNodeDist"))return;
            recovery<<timestamps[timeID]<<' '<<recov.lat<<' '<<recov.lon<<' '<<roadID<<'\n';
            ++timeID;
        }
    }
    recovery<<-id-1<<'\n';
    match<<'\n';
}

std::atomic<int> progress(0);
void print_progress(int lim) {
    while (true) {
        int current_progress = progress.load();
        int per = (100 * current_progress) / lim;
        std::cout << "\r[";
        for (int i = 0; i < 100; ++i) {
            int interval = per<10?1:(per<100?2:3);
            if(i < per)std::cout << "■";
            else if(i == per)std::cout << per << '%';
            else if(i >= per + interval+1)std::cout << ' ';
        }
        std::cout << "] (" << current_progress << "/" << lim << ")";
        std::cout << std::flush;
        if (current_progress >= lim) break;
        this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    cout << endl;
}

void process(int start, int end){
    for(int j=start;j<end;++j){
        solve(j, matchStream[j], recovStream[j]);
        ++progress;
    }
}

int main() {
    ios::sync_with_stdio(false);
    int m;
    ReadRoadNet(EDGEFILE,TYPEFILE,g,roads,inGrid);
    ReadTraces(TRACEFILE, m, traces, true, false);
    ReadVectors();
    m=5000;
    const int num_threads = 16;
    int chunk_size = (m + num_threads - 1) / num_threads;
    std::vector<std::thread> threads(num_threads);
    for (int i = 0; i < num_threads; ++i) {
        int start = i * chunk_size;
        int end = std::min(start + chunk_size, m);
        threads[i] = std::thread(process, start, end);
    }
    std::thread progress_thread(print_progress, m);
    for (auto& thread : threads) {
        thread.join();
    }
    progress_thread.join();

    clog<<"Output..."<<endl;
    ofstream recovery("Recovery.txt");
    recovery<<fixed<<setprecision(10);
    for(int i=0;i<m;++i)recovery<<recovStream[i].str();
    recovery.close();
    ofstream match("Full_Matched_Final.txt");
    match<<m<<'\n';
    for(int i=0;i<m;++i)match<<matchStream[i].str();
    match.close();
    return 0;
}

void ReadVectors(){
    clog<<"Reading Embedded Road Vectors..."<<endl;
    ifstream file("../../road_vectors.txt");
    int num_roads;
    file >> num_roads;
    for (int i = 0; i <= min(num_roads,PATH_NUM-1); ++i) {
        int road_id;
        file >> road_id;
        std::vector<float> vector(vec_len+3);  // Assuming the vector size is 8
        for (int j = 0; j < vec_len; ++j) file >> vector[j];
        road_vectors[road_id]=vector;
    }
    file.close();
}
