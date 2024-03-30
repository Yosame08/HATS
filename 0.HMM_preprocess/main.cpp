#include "definitions.h"
#undef TRACEFILE

#include <string>
const std::string mode = "train";
#define TRACEFILE ("../../"+mode+"_input.txt")

#include "structs.h"
#include "reads.h"
#include "maths.h"
#include <fstream>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <stack>
#include <queue>
#include <thread>
#include <atomic>
#include <cassert>
using namespace std;

G g;
Road roads[PATH_NUM];
vector<Trace>traces[524288];
ostringstream fullStream[524288], crossStream[524288];
//Feature feature[524288];
GridType inGrid;
vector<float>turnAngles[256][25];
vector<float>difDists[256][25];
vector<float>anchorError[256];

double DifDistProb(double dif) {
    return exp(-abs(dif) / BETA) / BETA;
}

SearchRes SearchRoad(int fromRoad, int toRoad, float toNodeDistA, float fromNodeDistB, const Trace &lastTr, const Trace &nowTr){
    vector<pair<PathNode,int>>seqPath;
    // A-star Algorithm
    bool vis[PATH_NUM+1]{};
    priority_queue<QInfo>q;
    const float greatCircle = lastTr.p.dist(nowTr.p);
    const unsigned long long span = nowTr.timestamp - lastTr.timestamp;
    seqPath.push_back({{fromRoad, lastTr.timestamp, toNodeDistA}, -1});
    q.push(QInfo{1, 0, toNodeDistA, greatCircle, FindAngle(fromRoad,0)-FindAngle(fromRoad,toNodeDistA)});
    if(q.top().angle<0)
        cout<<"(Bug) angle less than 0 [step 0: main.cpp]";
    assert(q.top().angle>=0);
    SearchRes result{-1};
    bool fin=false;

    while(!q.empty()&&!fin){
        const auto top=q.top();q.pop();
        const auto lastPath = seqPath[top.node].first;
        if(vis[lastPath.roadID])continue;
        vis[lastPath.roadID]=true;
        int node = roads[lastPath.roadID].to;
        for(auto to:g.node[node]){
            if(vis[to])continue;
            float angle = top.angle;
            angle += GetTurnAngle(seqPath[top.node].first.roadID, to);
            if(to==toRoad){
                float allLen = top.len + fromNodeDistB;
                if(allLen>=span*40)continue;
                seqPath.push_back({{to, lastPath.timestamp, (float)RoadLen(to)}, top.node});
                result = {DifDistProb(allLen - greatCircle), allLen, angle+FindAngle(toRoad, RoadLen(toRoad)-fromNodeDistB),(int)seqPath.size()-1};
                fin=true;
                break;
            }
            float totLen = top.len + RoadLen(to);
            if(totLen>=span*40||top.level>RECOVER_INTERVAL)continue;
            seqPath.push_back({{to, lastPath.timestamp, (float)RoadLen(to)}, top.node});
            q.push({top.level+1, (int)seqPath.size()-1, totLen, (float)roads[to].seg[0].line.startLL.dist(nowTr.p), angle+FindAngle(to,0)});
        }
    }
    // 搜索结束
    if(result.prob==-1)return result; // Search Failed
    result.path = new Path();
    stack<int>choose;
    for(int x=result.node;x!=0;x=seqPath[x].second)choose.push(x); // x!=0: ignore beginning
    while(!choose.empty()){
        result.path->push_back(seqPath[choose.top()].first);
        choose.pop();
    }
    double sumLen = toNodeDistA;
    for(int i=0;i<result.path->size();++i){
        //TODO: (Linear) Only when creating cases for training
        (*result.path)[i].timestamp = lastTr.timestamp + ((result.length==0)?span:(long long)(span*sumLen/result.length));
        sumLen+=(*result.path)[i].toNodeDist;
    }
    result.path->push_back({toRoad, nowTr.timestamp, (float)RoadLen(toRoad)-fromNodeDistB});
    return result;
}

std::atomic<int> unmatched(0);
void solve(int id, int thread, ostringstream &fullMatch, ostringstream &cross){
    auto myAssert = [id, &fullMatch](bool condition, const string& cause){
        if(condition)return true;
        string msg = string("Can't Match point to road at road id ")+ to_string(id)+string(" due to ")+cause;
        fullMatch<<msg<<'\n';
        ++unmatched;
        return false;
    };
    const auto& traceNow = traces[id];
    // Use Viterbi Alg. to perform matching - "prob" variable in SearchNode as a key value
    vector<Candidate>found;
    vector<SearchNode>search;
    vector<float>length, angles;
    int matched[traceNow.size()];
    FindRoad(40, 40, traceNow[0].p, found);
    if(!myAssert(!found.empty(), "Can't match point 0 to a road"))return;
    search.reserve(found.size());
    for(auto &x:found){
        search.push_back({x.prob,x.toNodeDist,0,x.roadID,-1,0,new Path{{x.roadID,traceNow[0].timestamp,x.toNodeDist}}});
        length.push_back(0), angles.push_back(0);
    }
    int oldBegin=0, oldEnd=(int)search.size()-1;

    for(int i=1;i<traceNow.size();++i){
        found.clear();
        FindRoad(40, 40, traceNow[i].p, found);
        if(!myAssert(!found.empty(), "Can't match point "+ to_string(i)+" to a road"))return;

        double maxProb=-1;
        for(auto &now:found){
            SearchNode maxNode{0};
            float maxNodeLen, maxAngle;
            for(int l=oldBegin;l<=oldEnd;++l) {
                SearchNode &old = search[l];
                Path *path;
                double traceProb;
                float ground, angle;
                if (old.roadID == now.roadID) {
                    path = new Path();
                    ground = old.toNodeDist - now.toNodeDist;
                    if (ground < -RECOVER_INTERVAL) continue;
                    traceProb = DifDistProb(traceNow[i - 1].p.dist(traceNow[i].p) - ground);
                    path->push_back({now.roadID, traceNow[i].timestamp, now.toNodeDist});
                    angle = FindAngle(now.roadID, now.toNodeDist) - FindAngle(old.roadID, old.toNodeDist);
                    if (angle < 0)angle = 0;
                } else {
                    SearchRes result = SearchRoad(old.roadID, now.roadID, old.toNodeDist,
                                                  RoadLen(now.roadID) - now.toNodeDist, traceNow[i - 1], traceNow[i]);
                    if (result.prob == -1)continue;
                    traceProb = result.prob, path = result.path;
                    ground = result.length, angle = result.angle;
                }
                // now.prob -> difference between GPS point and fullPath point
                double allProb = old.prob * now.prob * traceProb;
                if (allProb > maxNode.prob){
                    delete maxNode.path;
                    maxNode = {allProb, now.toNodeDist, ground, now.roadID, l, i, path};
                    maxNodeLen = ground, maxAngle = angle;
                }
                else delete path;
            }
            if(maxNode.prob>0){
                if(maxNode.prob>maxProb)maxProb=maxNode.prob;
                search.push_back(maxNode), length.push_back(maxNodeLen), angles.push_back(maxAngle);
            }
        }
        // Map imperfections can lead to HMM breaks, and in the preprocessing stage we filter out trajectories that cannot be matched directly
        if(!myAssert(search.size()-1!=oldEnd, "HMM break"))return;
        // Scale the maximum probability to 1 to avoid exceeding the precision of "double" if possible
        if(!myAssert(maxProb>0, "Too low probability"))return;
        oldBegin = oldEnd + 1, oldEnd = (int)search.size()-1;
        for(int x=oldBegin;x<=oldEnd;++x)search[x].prob/=maxProb;
    }

    // Find the node with max probability
    double maxProb=0;
    int outID;
    for(int i=oldBegin;i<=oldEnd;++i)if(search[i].prob>maxProb)maxProb=search[i].prob, outID=i;
    // Get the full trace
    Path fullPath;
    int recent24[25]{};

    vector<float>difSingle[25], turnStash[25], vels;
    while(true){
        const auto &node=search[outID];
        // 1. Stat: distance difference and angle turned
        if(node.prev!=-1){
            float totLen=0, totAngle=0;
            // Filtering anomalous data points on a road: far from an intersection but [traveling slowly / stopping]
            recent24[0]=outID;
            for(int i=1;i<=24;++i){
                if(!recent24[i])break;
                int &old=recent24[i];
                totLen += length[old];
                float dif = totLen;// - traceNow[node.pointID].p.dist(traceNow[search[old].pointID].p);
                totAngle += angles[old];
                difSingle[i].push_back(dif);
                turnStash[i].push_back(totAngle);
            }
            for(int i=24;i>=1;--i)recent24[i]=recent24[i-1];
            vels.push_back(recent24[1]);
        }

        float dist = traceNow[node.pointID].p.dist(FindLatLon(node.roadID, node.toNodeDist));
        anchorError[thread].push_back(dist);
        if(!myAssert(!node.path->empty(),"(Bug) Exists a node with no path at trace"+to_string(id)))return;
        for(int i=(int)node.path->size()-1;i>=0;--i){
            if(!myAssert((*node.path)[i].timestamp>0,"(Bug) Exists a path with negative timestamp at trace"+to_string(id)))return;
            fullPath.push_back((*node.path)[i]);
        }
        matched[node.pointID]=node.path->back().roadID;
        if(node.prev==-1)break;
        outID=node.prev;
    }
    // feature[id]={journeyLen/(float)journeyTime, 0, 0, id};
    int lastOut=-1,rear=int(fullPath.size())-1,nextOut=rear;
    // float journeyLen = -search[outID].toNodeDist; long long journeyTime = traceNow.back().timestamp-traceNow.front().timestamp;
    // bool ignoreInfo = false;
    cross<<fixed<<setprecision(3);
    for(int p=rear;p>=0;--p){
        auto &now=fullPath[p];
        if(now.roadID!=lastOut){
            lastOut=now.roadID;
            fullMatch << now.roadID << ' ';
        }
        //nextOut is the next point which refers to a different roadID
        if(p==nextOut){
            //ignoreInfo=false;
            for(int x=p-1;x>=0;--x){
                if(fullPath[x].roadID != now.roadID){
                    nextOut=x;
                    break;
                }
                // Filtering anomalous data points on a road: far from an intersection but [traveling slowly / stopping]
                //else if(fullPath[x].toNodeDist>300&fullPath[x+1].toNodeDist-fullPath[x].toNodeDist<fullPath[x].timestamp-fullPath[x+1].timestamp)ignoreInfo=true;
            }
            if(now.roadID == fullPath[nextOut].roadID)nextOut=-1;
        }
        assert(now.timestamp>0);
        if(p&&fullPath[p-1].timestamp==now.timestamp)continue; // if multiple points have the same timestamp, using the farthest
        //if(nextOut!=-1 && now.timestamp>0){
        //if(ignoreInfo)continue;
        if(p){
            long long hour=(now.timestamp%86400/3600+TIMEZONE)%24,sec=now.timestamp%3600;
            cross << id << ',' << now.roadID << ',' << (nextOut==-1?now.roadID:fullPath[nextOut].roadID) << ',' << hour << ',' << sec << ','
                  << now.toNodeDist << ',' << fullPath[p-1].timestamp - now.timestamp << '\n';
        }
        // old:       cross << id << ',' << now.roadID << ',' << fullPath[nextOut].roadID << ',' << hour << ',' << sec << ','
        //                  << now.toNodeDist << ',' << fullPath[nextOut].timestamp - now.timestamp << '\n';
    }
    fullMatch << '\n';
    for(int i=2;i<=24;++i){
        difDists[thread][i].insert(difDists[thread][i].end(),difSingle[i].begin(),difSingle[i].end());
        turnAngles[thread][i].insert(turnAngles[thread][i].end(),turnStash[i].begin(),turnStash[i].end());
    }
    for(auto &x:search)delete x.path;
    // Output matched GPS traces
}

std::atomic<int> progress(0);
void print_progress(int lim) {
    while (true) {
        int current_progress = progress.load();
        int per = (100 * current_progress) / lim;
        bool out = false;
        std::cout << "\r[";
        for (int i = 0; i < 101; i+=2) {
            int interval = per<10?1:(per<100?2:3);
            if(i < per)std::cout << "■";
            else if(i == per || !out){
                std::cout << per << '%';
                out = true;
            }
            else if(i > per + interval)std::cout << ' ';
        }
        std::cout << "] (" << current_progress << "/" << lim << ") [" << unmatched << " not matched]";
        std::cout << std::flush;
        if (current_progress >= lim) break;
        this_thread::sleep_for(std::chrono::milliseconds(2000));
    }
    cout << endl;
}

void process(int start, int end, int thread){
    for(int j=start;j<end;++j){
        solve(j, thread, fullStream[j], crossStream[j]);
        ++progress;
    }
}

int main() {
    ios::sync_with_stdio(false);
    ReadRoadNet(EDGEFILE,TYPEFILE,g,roads,inGrid);
    int m;
    ReadTraces(TRACEFILE, m, traces, true);
    cout<<"Read "<<m<<" trajectories"<<endl;
    const int num_threads = 16;
    int chunk_size = (m + num_threads - 1) / num_threads;
    std::vector<std::thread> threads(num_threads);
    for (int i = 0; i < num_threads; ++i) {
        int start = i * chunk_size;
        int end = std::min(start + chunk_size, m);
        threads[i] = std::thread(process, start, end, i);
    }
    std::thread progress_thread(print_progress, m);
    for (auto& thread : threads) {
        thread.join();
    }
    progress_thread.join();

    clog<<"Output..."<<endl;

    ofstream match("../../Intermediate/"+mode+"_full_matched.txt");
    match<<m<<'\n';
    for(int i=0;i<m;++i)match<<fullStream[i].str();

    ofstream cross("../../Intermediate/"+mode+"_traffic_data.csv");
    cross<<"traj_id,original_path_id,transition_path_id,hour,sec,distance,elapsed\n";
    for(int i=0;i<m;++i)cross<<crossStream[i].str();

    ofstream turnCount("../../Intermediate/"+mode+"_turn_cnt.txt"), difDistCount("../../Intermediate/"+mode+"_difDist_cnt.txt");
    turnCount<<fixed<<setprecision(1);
    difDistCount<<fixed<<setprecision(1);
    for(int j=2;j<=24;++j){
        turnCount<<j*15<<" Secs:\n";
        for(int i=0;i<num_threads;++i)for(auto x:turnAngles[i][j])turnCount<<(x*180/M_PI)<<' ';
        turnCount<<'\n';
        difDistCount<<j*15<<" Secs:\n";
        for(int i=0;i<num_threads;++i)for(auto x:difDists[i][j])difDistCount<<x<<' ';
        difDistCount<<'\n';
    }

    if(mode == "train"){
        ofstream anchor("../../Intermediate/anchorError.txt");
        anchor<<fixed<<setprecision(2);
        for(int i=0;i<num_threads;++i)for(auto x:anchorError[i])anchor<<x<<' ';
    }
    return 0;
}
