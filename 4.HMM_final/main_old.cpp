#include "graph.h"
#include "Structs.h"
#include "opCSV.h"
#include "myDefines.h"
#include "predict.h"
#include <fstream>
#include <iostream>
#include <stack>
#include <queue>
#include <unordered_map>
#include <iomanip>
#include <cstring>
#include <cassert>
#include <thread>
#include <mutex>
#include <atomic>
#include <cmath>
using namespace std;

G g;
Road roads[PATH_NUM];
vector<Trace>traces[262144];
ostringstream recovStream[262144], matchStream[262144];
unordered_map<int,unordered_map<int,vector<GridInfo>>>inGrid;
pair<double,double> cycles[PATH_NUM][24];

void ReadRoadnetOSM();
void ReadTraces_Real(int &m);
void ReadCycles();

mutex clog_mtx;
void safe_clog(const string& message) {
    lock_guard<mutex> lock(clog_mtx);
    clog << '\n' << message << endl;
}

double RoadLen(int roadID){
    return roads[roadID].seg.back().sumPrev;
}

double PtMatchProb(double dist){
    static const double coefficient = 1 / (SIGZ * sqrt(M_2_PI));
    double param = dist / SIGZ;
    return exp(-(param * param * 0.5)) * coefficient;
}

double DifDistProb(double dif) {
    return sqrt(exp(-dif / BETA) / BETA);
}

double SearchDifDistProb(double dif) {
    if(dif<0)return 1;
    double ans = sqrt(exp(-dif / BETA) / BETA);
    return ans==0 ? 1e-200 : ans;
}

double TypeChangeProb(int _old, int _new){
    return pow(2, _new-_old-6);
}

void FindRoad(Point &p, vector<Candidate>&found){
    int gridX= int(p.x / GRIDSIZE), gridY= int(p.y / GRIDSIZE);
    for(int lim = 35;found.empty()&&lim<=350;lim+=35){
        for(int x= gridX-25; x <= gridX+25; ++x)for(int y= gridY-25; y <= gridY+25; ++y){
            if(inGrid.find(x)==inGrid.end()||inGrid[x].find(y)==inGrid[x].end())continue;
            for(auto &i:inGrid[x][y]){
                Point cross;
                Segments &nowSeg = roads[i.roadID].seg[i.segID];
                double dist = DistPointSeg(nowSeg.line, p, cross);
                if(dist>lim)continue;
                found.push_back({i.roadID,nowSeg.sumAfter-Distance(cross,nowSeg.line.start), PtMatchProb(dist)});
            }
        }
    }
}

double Transition(double dire){
    return exp(-pow(dire,4));
}

double TimeUpJudge(long long over){
    if(over<0)return 1;
    double ans=sqrt(exp(-over));
    return ans==0?1e-200:ans;
}

SearchRes SearchRoad(int fromRoad, int toRoad, double toNodeDistA, double fromNodeDistB, const Trace &lastTr, const Trace &nowTr){
    // seqPath is to restore nodes searched. Every node restore information about current road and which node is previous road
    vector<pair<PathNode,int>>seqPath; seqPath.reserve(65536);
    // Use DP to simplify searching process. Like Dijkstra but use "probability" as key for sort
    bool vis[PATH_NUM]; memset(vis,0,sizeof vis); priority_queue<QueueInfo>q;
    // Initialize some const Value to prune search tree
    const double greatCircle = Distance(lastTr.p, nowTr.p); const unsigned long long span = nowTr.timestamp - lastTr.timestamp;
    // Set starting state
    seqPath.push_back({{fromRoad, lastTr.timestamp, toNodeDistA}, -1});
    q.push(QueueInfo{1, (int)seqPath.size()-1, 1, toNodeDistA});
    SearchRes result{-1};
    int update=0;
    while(!q.empty()){
        const auto top=q.top();q.pop();
        const auto &lastPath = seqPath[top.node].first;
        if(vis[lastPath.roadID])continue;
        vis[lastPath.roadID]=true;
        int node = roads[lastPath.roadID].to;
        for(auto to:g.node[node]){
            if(vis[to])continue;
            long long timestamp = lastPath.timestamp + TurnPrediction(lastPath.roadID, to, lastPath.toNodeDist, lastPath.timestamp);
            double totLen = top.len + RoadLen(to);
            double tranProb = top.accuProb / SearchDifDistProb(top.len - greatCircle) * SearchDifDistProb(totLen - greatCircle)
                                           / TimeUpJudge(lastPath.timestamp-nowTr.timestamp) * TimeUpJudge(timestamp-nowTr.timestamp)
                                           * TypeChangeProb(roads[lastPath.roadID].level, roads[to].level);
            // 计算转弯概率，忽略长6m以下的道路
            for(int oldnode=top.node;oldnode!=-1;oldnode=seqPath[oldnode].second){
                const int &roadID = seqPath[oldnode].first.roadID;
                if(RoadLen(roadID)>6){
                    double &fromDirect = roads[roadID].seg.back().line.dire, &toDirect = roads[to].seg[0].line.dire;
                    double &direBig = fromDirect>toDirect ? fromDirect : toDirect, &direSmall = fromDirect>toDirect ? toDirect : fromDirect;
                    tranProb*=Transition(min(direBig - direSmall, direSmall - direBig + 2 * M_PI));
                    break;
                }
            }
            // 找到目的地，计算概率
            if(to==toRoad){
                double allLen = top.len + fromNodeDistB;
                // 如果是终点，没有走到头，修改计算的概率
                double outProb = tranProb / SearchDifDistProb(totLen - greatCircle) * SearchDifDistProb(allLen - greatCircle);
                if(outProb>result.prob){
                    seqPath.push_back({{to, lastPath.timestamp, RoadLen(toRoad)}, top.node});
                    result = {outProb, allLen, (int)seqPath.size()-1};
                }
                if(++update>2)break;
                continue;
            }
            // 检查应该入队还是被剪枝
            if(seqPath.size()<(1<<21)){
                // 各种剪枝：最多执行span/3步，也就是最快允许每3秒换一条路段
                if(top.level + 1 > span/3)continue;
                // span/4步后，比起点前更靠近目的地（直线）
                if(top.level>span/4 && greatCircle<Distance(nowTr.p, roads[to].seg.back().line.end))continue;
                // 车速极快
                if(totLen > span*36)continue;
                // 通过剪枝，入队
                seqPath.push_back({{to, timestamp, RoadLen(to)}, top.node});
                q.push({top.level + 1, (int)seqPath.size()-1, tranProb, totLen});
                if(seqPath.size()%(1<<20)==0)safe_clog(string("seqPath.size:")+ to_string(seqPath.size()));
            }
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
    if(result.length>0){
        double limit = span * ((result.length - fromNodeDistB)/(double)result.length); // time to the last turn (linear)
        long long totTime = result.path.back().timestamp - lastTr.timestamp;
        double per = limit/totTime;
        for(int i=1;i<result.path.size();++i)result.path[i].timestamp = lastTr.timestamp + (result.path[i].timestamp - lastTr.timestamp) * per;
    }else{
        for(int i=1;i<result.path.size();++i)result.path[i].timestamp=lastTr.timestamp*span*(double(i)/result.path.size());
    }
    result.path.push_back({toRoad, nowTr.timestamp, RoadLen(toRoad)-fromNodeDistB});
    return result;
}

void solve(int id, ostringstream &match, ostringstream &recovery){
    auto beginTime = clock();
    auto myAssert = [&](bool condition, const string& cause){
        if(condition)return true;
        recovery<<"Failed\n"<<-id-1<<'\n';
        safe_clog(string("Can't Match point to road at road id ")+ to_string(id) +string(" due to ")+cause);
        return false;
    };
    // Use Viterbi Alg. to perform matching - "prob" variable in SearchNode as a key value
    vector<Candidate>found;
    vector<SearchNode>search;
    vector<long long>timestamps;
    timestamps.push_back(traces[id][0].timestamp);
    FindRoad(traces[id][0].p, found);
    if(!myAssert(!found.empty(), "Can't match point 0 to a road"))return;
    search.reserve(found.size());
    for(auto &x:found)search.push_back({x.prob,x.toNodeDist,x.roadID,-1,0,{{x.roadID,traces[id][0].timestamp,x.toNodeDist}}});
    int oldBegin=0, oldEnd=(int)search.size()-1;

    for(int i=1;i<traces[id].size();++i){
        for(long long t=timestamps.back()+RECOVER_INTERVAL;t<traces[id][i].timestamp;t+=RECOVER_INTERVAL)timestamps.push_back(t);
        timestamps.push_back(traces[id][i].timestamp);
        found.clear();
        FindRoad(traces[id][i].p, found);
        if(!myAssert(!found.empty(), "Can't match point "+ to_string(i)+" to a road"))return;

        for(auto &now:found){
            SearchNode maxNode{0};
            for(int l=oldBegin;l<=oldEnd;++l){
                SearchNode &old = search[l];
                Path path;
                double traceProb;
                if(old.roadID == now.roadID){
                    double ground = old.toNodeDist - now.toNodeDist;
                    traceProb = DifDistProb(abs(Distance(traces[id][i - 1].p, traces[id][i].p) - ground));
                    path.push_back({now.roadID,traces[id][i].timestamp,now.toNodeDist});
                }
                else{
                    SearchRes result = SearchRoad(old.roadID, now.roadID, old.toNodeDist,
                                                  RoadLen(now.roadID) - now.toNodeDist, traces[id][i-1], traces[id][i]);
                    if(result.prob==-1)continue;
                    traceProb=result.prob;
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
                search.push_back({now.prob, now.toNodeDist, now.roadID, mxPoint, i, {{now.roadID,traces[id][i].timestamp,now.toNodeDist}}});
        }
        oldBegin = oldEnd + 1, oldEnd = (int)search.size()-1;
        double maxProb=-1;
        for(int x=oldBegin;x<=oldEnd;++x)if(search[x].prob>maxProb)maxProb=search[x].prob;
        if(maxProb!=0)for(int x=oldBegin;x<=oldEnd;++x)search[x].prob/=maxProb;
    }
    auto inferTime = clock()-beginTime;
    beginTime = clock();
    double maxProb=0;
    int outID;
    for(int i=oldBegin;i<=oldEnd;++i)if(search[i].prob>maxProb)maxProb=search[i].prob, outID=i;
    Path matched;
    PathNode interpolation{-1,-1,-1};
    while(outID!=-1){
        auto &node=search[outID];
        if(!myAssert(!node.path.empty(),"(Bug) Exists a node with no path at trace"+to_string(id)))return;
        for(int i=(int)node.path.size()-1;i>=0;--i)matched.push_back(node.path[i]);
        outID=node.prev;
    }
    int lastOut=-1,rear=int(matched.size())-1,nextOut=rear,timeID=0,lastMatchDist=-1;
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
            int roadID; double toNodeDist;
            if(timestamps[timeID]==now.timestamp){
                roadID=now.roadID;
                toNodeDist=now.toNodeDist;
            }else{
                //interpolation
                auto &last = matched[p+1].timestamp<=interpolation.timestamp?matched[p+1]:interpolation;
                long long turnTime = last.timestamp + TurnPrediction(last.roadID, now.roadID, last.toNodeDist, last.timestamp);
                if(turnTime>timestamps[timeID]){ // not turn yet
                    roadID=last.roadID;
                    double len = VelPrediction(roadID, last.toNodeDist, last.timestamp) / RECOVER_INTERVAL * (timestamps[timeID]-turnTime) ;
                    if(len<0)len=0;
                    toNodeDist= len > last.toNodeDist ? 0 : last.toNodeDist - len;
                }
                else{
                    roadID=now.roadID;
                    double len = VelPrediction(roadID, RoadLen(now.roadID), last.timestamp+turnTime) / RECOVER_INTERVAL * (timestamps[timeID]-last.timestamp-turnTime) ;
                    if(len<0)len=0;
                    toNodeDist= len > RoadLen(now.roadID) ? 0 : RoadLen(now.roadID) - len;
                }
            }
            Point recov{-1024,-1024};
            for(int i=int(roads[roadID].seg.size())-1;i>=0;--i){
                auto &seg=roads[roadID].seg[i];
                if(seg.sumAfter >= toNodeDist - EPS){
                    double per2 = seg.line.len>0 ? (seg.sumAfter - toNodeDist) / seg.line.len : 1;
                    recov=seg.line.startLL+(seg.line.endLL-seg.line.startLL)*per2;
                    break;
                }
            }
            interpolation = PathNode{roadID,timestamps[timeID],toNodeDist};
            if(!myAssert(!(recov==Point(-1024,-1024)), "(Bug) Bad valud: toNodeDist"))return;
            recovery<<timestamps[timeID]<<' '<<recov.x<<' '<<recov.y<<' '<<roadID<<'\n';
            ++timeID;
        }
    }
    recovery<<-id-1<<'\n';
    match<<'\n';
    //safe_clog("[ID:"+to_string(id)+" size:"+to_string(traces[id].size())+"] Inference:"+ to_string(inferTime/(double)CLOCKS_PER_SEC)+" Traceback:"+
    //                                                                                                          to_string((clock()-beginTime)/(double)CLOCKS_PER_SEC));
}

std::atomic<int> progress(0);
void print_progress(int lim) {
    while (true) {
        int current_progress = progress.load();
        cout << "\rProgress: " << current_progress << "/" << lim << std::flush;
        if (current_progress >= lim) break;
        this_thread::sleep_for(chrono::milliseconds(1000));
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
    ReadRoadnetOSM();
    int m;
    ReadTraces_Real(m);
    ReadCycles();
    m=400;
    const int num_threads = 15;
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
    recovery<<fixed<<setprecision(8);
    for(int i=0;i<m;++i)recovery<<recovStream[i].str();
    recovery.close();
    ofstream match("Full_Matched_Final.txt");
    match<<m<<'\n';
    for(int i=0;i<m;++i)match<<matchStream[i].str();
    match.close();
    return 0;
}

void ReadRoadnetOSM(){
    clog<<"Reading RoadNet..."<<endl;
    ifstream edgeFile(EDGEFILE);
    string line;
    while(std::getline(edgeFile, line)){
        std::istringstream iss(line);
        int id, from, to, num;
        iss >> id >> from >> to >> num;
        // 特判对地图匹配没用的路
        if(num == 1){
            double skip;
            iss>>skip>>skip;
            continue;
        }
        else if(num == 2 && from == to){
            double skip;
            iss>>skip>>skip>>skip>>skip;
            continue;
        }
        g.connect(from,id);
        roads[id]=Road{-1,from,to};
        Point last, lastLL;
        for(int j=0;j<num;++j){
            double lat, lon;
            iss>>lat>>lon;
            auto res = latLonToXY(lat,lon);
            Point now{res.first, res.second};
            if(j){
                Point mid = (last+now)/2;
                inGrid[int(mid.x / GRIDSIZE)][int(mid.y / GRIDSIZE)].push_back({id, j - 1});
                roads[id].seg.push_back(Segments{Line{last, now, lastLL, {lat,lon}}, 0, 0});
            }
            last = now, lastLL={lat,lon};
        }
        auto &lines = roads[id].seg;
        lines[0].sumPrev=lines[0].line.len;
        lines[lines.size()-1].sumAfter=lines[lines.size()-1].line.len;
        for(int x=1;x<lines.size();++x)lines[x].sumPrev=lines[x-1].sumPrev+lines[x].line.len;
        for(int x=int(lines.size())-2;x>=0;--x)lines[x].sumAfter=lines[x+1].sumAfter+lines[x].line.len;
    }
    edgeFile.close();

    std::ifstream typeFile(TYPEFILE);
    while(std::getline(typeFile, line)){
        std::istringstream iss(line);
        int id, type;
        string typeName;
        iss >> id >> typeName >> type;
        roads[id].level=type;
    }
}

void ReadTraces_Real(int &num){
    clog<<"Reading Traces..."<<endl;
    ifstream tr(TRACEFILE);
    tr>>num;
    for(int x=0;x<num;++x){
        if(x%8192==0)clog<<x<<'\r'<<flush;
        while(true){
            long long stamp, roadID;
            double lat, lon;
            tr>>stamp;
            if(stamp<1048576)break;
            tr>>lat>>lon>>roadID;
            auto res = latLonToXY(lat,lon);
            traces[x].push_back({{res.first,res.second},stamp, lat, lon});
        }
    }
}

void ReadCycles(){
    clog<<"Reading Traffic Data..."<<endl;
    CSVFile lights("../../train_light_cycle_est.csv");
    for(auto &line:lights)
        cycles[(int)line["RoadID"]][(int)line["hour"]]=make_pair(line["BestCycle"],line["SecondCycle"]);
}
