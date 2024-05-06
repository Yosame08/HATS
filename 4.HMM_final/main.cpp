#include "definitions.h"
#include "structs.h"
#include "reads.h"
#include "predict.h"
#include "FuncEst.h"
#include "funcIO.h"
#include <fstream>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <stack>
#include <queue>
#include <thread>
#include <atomic>
#include <unordered_set>
#include <map>
#include <cassert>
#include <cstring>
using namespace std;

double sigz;
float roadSpeed[7];
FunctionFit parTurn(PARAMTURN), parLenPos(PARAMLENPOS), parLenNeg(PARAMLENNEG);
G g;
Road roads[PATH_NUM];
vector<Trace>traces[524288];
ostringstream recovStream[524288], matchStream[524288];
GridType inGrid;
vector<float> road_vectors[PATH_NUM];
void ReadVectors();

/// @brief Find the index of time interval
/// @param out l, r
void FindIndex(unsigned long long time, int &l, int &r){
    const auto &times = parTurn.times;
    auto f = lower_bound(times.begin(), times.end(), time);
    if(f==times.end()) l = r = int(times.size()-1); // time > maximum interval
    else if(f==times.begin()&&time<*f) l = r = 0; // time < minimum interval
    else if(time==*f) l = r = int(f - times.begin());
    else{
        r = int(f-times.begin());
        l = r - 1;
    }
}

/// @param angle RAD
/// @brief Convert radians to angles and predict with the fitted function
double AngleProb(double angle, int time){
    //assert(angle==angle);
    int l, r;
    FindIndex(time, l, r);
    if(l == r)return parTurn.Estimate_wrap(angle*180/M_PI, l);
    return (parTurn.Estimate_wrap(angle*180/M_PI, l)
            +parTurn.Estimate_wrap(angle*180/M_PI, r))/2;
}

double DifDistProb(double dif, int time) {
    //assert(dif==dif);
    const auto &use = dif<0?parLenNeg:parLenPos;
    int l, r;
    FindIndex(time, l, r);
    if(l == r)return use.Estimate_wrap(dif,l);
    return (use.Estimate_wrap(dif,l)+use.Estimate_wrap(dif,r))/2;
}

double SearchDifDistProb(double dif, int time) {
    //assert(dif==dif);
    if(dif<0)return 1;
    return DifDistProb(dif, time);
}

double MedianSpeedProb(float len, float estTime){
    static const float maxSpeed = *max_element(roadSpeed, roadSpeed+7);
    float mini = len / maxSpeed;
    return mini / estTime; // Relative probability is ensured - double time, half probability
}

SearchRes SearchRoad(const SearchNode& old, const Candidate& now, const Trace &lastTr, const Trace &nowTr, double accuProb, double maxProb){
    const int &fromRoad = old.roadID, &toRoad = now.roadID;
    const float &toNodeDistA = old.toNodeDist, fromNodeDistB = RoadLen(toRoad) - now.toNodeDist;
    // seqPath is to restore nodes searched. Every node restore information about current road and which node is previous road
    vector<QueueInfo2>seqPath;
    // Use DP to simplify searching process, which is like Dijkstra but use "probability" as the key
    BitInt vis;
    priority_queue<QueueInfo>q;
    // Initialize some const Value to prune search tree
    const auto greatCircle = (float)lastTr.p.dist(nowTr.p);
    const int span = int(nowTr.timestamp - lastTr.timestamp);
    int seqSize = 1;
    // Set starting state
    seqPath.push_back({PathNode{fromRoad, lastTr.timestamp, toNodeDistA}, -1, 0,
                       toNodeDistA, FindAngle(fromRoad,0)-FindAngle(fromRoad,toNodeDistA), toNodeDistA/roadSpeed[roads[fromRoad].level]});
    q.push(QueueInfo{1, 0});
    SearchRes result{-1,0,0,0, nullptr};

    while(!q.empty()){
        int lastNode = q.top().node;
        const auto lastInfo = seqPath[lastNode];
        int atRoad = lastInfo.pNode.roadID;
        if(atRoad == toRoad && seqPath.size()!=1){
            result = {q.top().accuProb, lastInfo.len, lastInfo.angle, lastNode};
            break;
        }
        q.pop();
        if(vis.chk(atRoad))continue;
        vis.set(atRoad);
        int node = roads[atRoad].to;
        for(const auto &to:g.node[node]){
            if(vis.chk(to))continue;

            float angle = lastInfo.angle;
            if(RoadLen(to)>=10){
                int angleNode = lastNode;
                for(;angleNode>=0 && RoadLen(seqPath[angleNode].pNode.roadID)<10; angleNode = seqPath[angleNode].prev);
                if(angleNode>=0){
                    if(angleNode == lastNode)angle += GetTurnAngle(atRoad, to);
                    else angle += DiscontinuousAngle(seqPath[angleNode].pNode.roadID, to);
                }
            }

            // Found the destination
            if(to==toRoad){
                float allLen = lastInfo.len + fromNodeDistB;
                if(allLen >= float(span) * SPEEDLIM)continue;
                float consume = fromNodeDistB / roadSpeed[roads[toRoad].level];
                angle += FindAngle(toRoad, RoadLen(toRoad)-fromNodeDistB);
                double outProb = DifDistProb(allLen - greatCircle, span) * AngleProb(angle, span)
                        * MedianSpeedProb(allLen, lastInfo.estTime+consume);
                if(outProb > result.prob){
                    seqPath.push_back({PathNode{to, lastInfo.pNode.timestamp, (float)RoadLen(toRoad)},
                                       lastNode, lastInfo.level+1, allLen, angle});
                    q.push({outProb, seqSize++});
                }
                continue;
            }
            float passLen = RoadLen(to);
            float totLen = lastInfo.len + passLen;
            // enqueue or be pruned
            if(lastInfo.level > span/2)continue;
            if(lastInfo.level > span/3 && greatCircle < nowTr.p.dist(roads[to].seg.back().line.endLL))continue;
            if(totLen >= float(span) * SPEEDLIM)continue;

            angle += FindAngle(to,0);
            float consume = lastInfo.estTime + passLen / roadSpeed[roads[to].level];
            seqPath.push_back({PathNode{to, lastInfo.pNode.timestamp, (float)RoadLen(to)},
                               lastNode, lastInfo.level+1, totLen, angle, consume});
            q.push({SearchDifDistProb(totLen - greatCircle + roads[to].seg.back().line.endLL.dist(nowTr.p), span) *
                AngleProb(angle, span) *
                MedianSpeedProb(totLen, consume), seqSize++}); //
        }
    }
    // search end
    if(result.prob * accuProb <= maxProb)return {result.prob,-1}; // Search Failed
    result.path = new Path();
    stack<int>choose;
    for(int x=result.node;x!=-1;x=seqPath[x].prev)choose.push(x);
    while(!choose.empty()){
        result.path->push_back(seqPath[choose.top()].pNode);
        choose.pop();
    }
    result.path->push_back({toRoad, nowTr.timestamp, (float)RoadLen(toRoad)-fromNodeDistB});
    return result;
}

std::atomic<clock_t>sPhaseTM, iPhaseTM;
void solve(int id){
    //if(id<4)return;
    auto myAssert = [&](bool condition, const string& cause){
        if(condition)return true;
        recovStream[id]<<"Failed\n"<<-id-1<<'\n';
        safe_cout(string("Can't Match point to road at path id ")+ to_string(id) +string(" due to ")+cause);
        return false;
    };
    auto beginTM = clock();
    const auto& traceNow = traces[id];

    // Use Viterbi Alg. to perform matching - "prob" variable in SearchNode as a key value
    vector<Candidate>found;
    FindRoad(80, 400, traceNow[0].p, found, sigz);
    if(!myAssert(!found.empty(), "Can't match point 0 to a road"))return;

    vector<SearchNode>search;
    search.reserve(found.size());
    for(auto &x:found)
        search.push_back({x.prob, x.toNodeDist, 0, x.roadID, -1, 0, new Path{{x.roadID, traceNow[0].timestamp, x.toNodeDist}}});

    int oldBegin=0, oldEnd=(int)search.size()-1;
    unordered_set<unsigned long long>noPath;

    for(int i=1;i<traceNow.size();++i){

        found.clear();
        FindRoad(80, 400, traceNow[i].p, found, sigz);
        if(!myAssert(!found.empty(), "Can't match point "+ to_string(i)+" to a road"))return;

        for(auto &now:found){
            SearchNode maxNode{0,0,0,0,0,0,nullptr};
            for(int l=oldBegin;l<=oldEnd;++l){
                SearchNode &old = search[l];
                if(traceNow[i].timestamp - traceNow[i-1].timestamp>=65536) throw "Too long interval of trajectory!";
                unsigned long long hashed = now.roadID | ((unsigned long long)old.roadID << 32llu);
                if(noPath.count(hashed))continue;
                double traceProb, allProb;
                if(old.roadID == now.roadID){
                    float ground = old.toNodeDist - now.toNodeDist, angle = FindAngle(now.roadID, now.toNodeDist) - FindAngle(old.roadID, old.toNodeDist);
                    assert(traceNow[i - 1].p.dist(traceNow[i].p) == traceNow[i - 1].p.dist(traceNow[i].p));
                    traceProb = DifDistProb(ground - traceNow[i - 1].p.dist(traceNow[i].p), int(traceNow[i].timestamp-traceNow[i-1].timestamp)) * //
                                AngleProb(angle, int(traceNow[i].timestamp-traceNow[i-1].timestamp));
                    allProb = old.prob * now.prob * traceProb;
                    if(allProb > maxNode.prob) {
                        delete maxNode.path;
                        maxNode = {allProb, now.toNodeDist, ground, now.roadID, l, i, new Path()};
                        maxNode.path->push_back({old.roadID, traceNow[i - 1].timestamp, old.toNodeDist});
                        maxNode.path->push_back({now.roadID, traceNow[i].timestamp, now.toNodeDist});
                    }
                }
                else{
                    double accuProb = old.prob * now.prob;
                    SearchRes result = SearchRoad(old, now, traceNow[i-1], traceNow[i], accuProb, maxNode.prob);
                    if(result.length==-1){
                        if(result.prob==-1)noPath.insert(hashed);
                        continue;
                    }
                    allProb = accuProb * result.prob;
                    delete maxNode.path;
                    maxNode = {allProb, now.toNodeDist, result.length, now.roadID, l, i, result.path};
                }
            }
            if(maxNode.prob>0)search.push_back(maxNode);
        }
        if(search.size()-1==oldEnd){ // HMM break
            safe_cout(string("HMM break at PATH ")+ to_string(id) + string(" trace ")+ to_string(i));
            int mxPoint=0;double mxProb=-1;
            for (int k = oldBegin; k <= oldEnd; ++k)if(search[k].prob > mxProb)mxProb=search[k].prob, mxPoint=k;
            for(auto &now:found)
                search.push_back({now.prob, now.toNodeDist, 0, now.roadID, mxPoint, i, new Path{{now.roadID,traceNow[i].timestamp,now.toNodeDist}}});
        }
        oldBegin = oldEnd + 1, oldEnd = (int)search.size()-1;
        double maxProb=-1;
        for(int x=oldBegin;x<=oldEnd;++x)if(search[x].prob>maxProb)maxProb=search[x].prob;
        if(maxProb!=0)for(int x=oldBegin;x<=oldEnd;++x)search[x].prob/=maxProb;
    }
    auto SPH = clock()-beginTM;
    beginTM = clock();
    double maxProb=0;
    int outID;
    for(int i=oldBegin;i<=oldEnd;++i)if(search[i].prob>maxProb)maxProb=search[i].prob, outID=i;
    PointLL lastPoint = FindLatLon(search[outID].roadID, search[outID].toNodeDist);
    int lastID = search[outID].roadID;
    // Stat
    map<int,float>journeyLen;
    deque<int>fullPath;
    for(int id = outID; id != -1; id = search[id].prev){
        for(auto it=search[id].path->rbegin(); it!=search[id].path->rend(); ++it)fullPath.push_front(it->roadID);
        journeyLen[id] = search[id].length;
    }
    float totLen = 0;
    for(auto &x:journeyLen){
        float t = x.second;
        x.second = totLen;
        totLen += t;
    }
    // Interpolation
    vector<Path>result;
    Path* nxtPath = nullptr;
    while(search[outID].prev!=-1){
        auto &node=search[outID];
        if(!myAssert(node.path->size()>=2,"(Bug) Exists a node with no path at trace"+to_string(id)))return;

        auto &last=search[node.prev];
        result.push_back(Path{PathNode{last.roadID,traceNow[last.pointID].timestamp,last.toNodeDist}});
        vector<float> *maxPredVel=nullptr;
        long long minDif = 0, tm = traceNow[last.pointID].timestamp;//, st = traceNow[0].timestamp;
        //const long long journey = traceNow.back().timestamp-traceNow.front().timestamp;
        float toNodeDist = last.toNodeDist, passed = 0, vel;// = beginVel;
        int roadID = last.roadID, toID = (*node.path)[1].roadID, index=1;
        auto *predVel = new vector<float>{};
        while(index < node.path->size() - 1){
            while (toNodeDist > 0) {
                vel = VelPrediction(roadID, toID, toNodeDist, tm, (journeyLen[outID] + passed)/totLen); //, (tm+1-st)/(double)journey)
                tm+=1, toNodeDist -= vel, predVel->push_back(vel), passed += vel;
            }
            while (toNodeDist <= 0 && index < node.path->size() - 1){
                roadID = toID;
                toNodeDist += RoadLen(toID);
                toID = (*node.path)[++index].roadID;
            }
        }
        float needPass = toNodeDist - node.toNodeDist;
        if(nxtPath!= nullptr){
            for(auto x:*nxtPath){
                if(x.roadID!=toID){
                    toID=x.roadID;
                    break;
                }
            }
        }
        while (needPass > 0) {
            vel = VelPrediction(roadID, toID, toNodeDist, tm, (journeyLen[outID] + passed)/totLen); // , (tm+1-st)/(double)journey)
            ++tm, toNodeDist -= vel, needPass -= vel, predVel->push_back(vel), passed += vel;
        }
        if (abs(tm - traceNow[node.pointID].timestamp) < abs(minDif - traceNow[node.pointID].timestamp)) {
            minDif = tm;
            delete maxPredVel;
            maxPredVel = predVel;
        }
        else delete predVel;

        if(minDif == traceNow[last.pointID].timestamp){
            for(auto t = traceNow[last.pointID].timestamp+RECOVER_INTERVAL;t<traceNow[node.pointID].timestamp;t+=RECOVER_INTERVAL)
                result.back().push_back(PathNode{last.roadID, t, last.toNodeDist});
        }
        else{
            double scale=double(traceNow[node.pointID].timestamp-traceNow[last.pointID].timestamp)/double(minDif-traceNow[last.pointID].timestamp), accu=0;
            toNodeDist = last.toNodeDist;
            roadID = last.roadID, index=1;
            auto update = [&](double dist){
                toNodeDist -= (float)dist;
                while(toNodeDist <= 0 && index < node.path->size() - 1){
                    toNodeDist += RoadLen((*node.path)[index].roadID);
                    roadID = (*node.path)[index++].roadID;
                }
                if(toNodeDist<0)toNodeDist=0;
            };
            for(auto x:*maxPredVel){
                double used = 0;
                while(accu+scale*(1-used)>=RECOVER_INTERVAL-EPS){
                    double ratio = (RECOVER_INTERVAL - accu) / scale;
                    if(ratio > 1)ratio=1;
                    used += ratio;
                    accu += scale * ratio;
                    update(x * ratio);
                    accu-=RECOVER_INTERVAL;
                    result.back().push_back(PathNode{roadID, result.back().back().timestamp+RECOVER_INTERVAL, toNodeDist});
                }
                update(x*(1-used));
                accu += scale*(1-used);
            }
        }
        nxtPath=node.path;
        outID=node.prev;
    }
    long long printStamp = 0;
    recovStream[id]<<fixed<<setprecision(8);
    for(auto tr=result.rbegin();tr!=result.rend();++tr){
        for(auto &x:*tr){
            if(x.timestamp==printStamp)continue;
            else printStamp=x.timestamp;
            PointLL recov=FindLatLon(x.roadID,x.toNodeDist);
            recovStream[id]<<x.timestamp<<' '<<recov.lat<<' '<<recov.lon<<' '<<x.roadID<<'\n';
        }
    }
    if(traceNow.back().timestamp!=printStamp)
        recovStream[id]<<traceNow.back().timestamp<<' '<<lastPoint.lat<<' '<<lastPoint.lon<<' '<<lastID<<'\n';
    recovStream[id]<<-id-1<<'\n';
    int lastRoad = -1;
    while(!fullPath.empty()){
        int ID = fullPath.front();
        if(ID==lastRoad){
            fullPath.pop_front();
            continue;
        }
        matchStream[id]<<ID<<' ';
        lastRoad = ID;
        fullPath.pop_front();
    }
    matchStream[id]<<'\n';
    auto IPH = clock()-beginTM;
    sPhaseTM += SPH, iPhaseTM += IPH;
    for(auto &x:search)delete x.path;
}

std::atomic<int> progress(0);
void print_progress(int lim, int thread) {
    while (true) {
        int current = progress.load();
        int per = (100 * current) / lim;
        bool out = false;
        std::cout << "\r[";
        for (int i = 0; i < 101; i += 2) {
            int interval = per<10?1:(per<100?2:3);
            if(i < per)std::cout << "■";
            else if(i == per || !out){
                std::cout << per << '%';
                out = true;
            }
            else if(i >= per + interval+1)std::cout << ' ';
        }
        std::cout << "] (" << current << "/" << lim << ") ";
        if(!current)current=1;
        std::cout << "[Search " << (double)sPhaseTM/CLOCKS_PER_SEC/current/thread << "s, Interpolation " << (double)iPhaseTM/CLOCKS_PER_SEC/current/thread << "s]";
        std::cout << std::flush;
        if (current >= lim) break;
        this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    cout << endl;
}

void process(int start, int end){
    for(int j=start;j<end;++j){
        solve(j);
        ++progress;
    }
}

int main(int argc, char* argv[]) {
    ios::sync_with_stdio(false);
    ifstream param("../Intermediate/train_params.param");
    string ignoreStr;
    for(int i=0;i<7;++i)param>>ignoreStr>>roadSpeed[i];
    //in case no data in training set
    for(int i=1;i<6;++i)if(roadSpeed[i]==-1)roadSpeed[i]=(roadSpeed[i-1]+roadSpeed[i+1])/2;
    if(roadSpeed[0]==-1)roadSpeed[0]=roadSpeed[1]-(roadSpeed[2]-roadSpeed[1]);
    if(roadSpeed[6]==-1)roadSpeed[6]=roadSpeed[5]+(roadSpeed[5]-roadSpeed[4]);

    param>>ignoreStr>>sigz;
    int m;
    ReadRoadNet(EDGEFILE,TYPEFILE,g,roads,inGrid);
    ReadTraces(TRACEFILE, m, traces, true, false);
    ReadVectors();

    int num_threads = 8; // default value
    string recovFN = "recovery";
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-th") == 0 && i + 1 < argc) {
            num_threads = std::stoi(argv[i + 1]);
            if (num_threads < 1)num_threads = 1;
            ++i; // skip next argument
        }
        else if (strcmp(argv[i], "-fn") == 0 && i + 1 < argc) {
            recovFN = argv[i + 1];
            ++i; // skip next argument
        }
        else cout << "redundant argument: " << argv[i] << endl;
    }
    int chunk_size = (m + num_threads - 1) / num_threads;
    std::vector<std::thread> threads(num_threads);
    for (int i = 0; i < num_threads; ++i) {
        int start = i * chunk_size;
        int end = std::min(start + chunk_size, m);
        threads[i] = std::thread(process, start, end);
    }
    std::thread progress_thread(print_progress, m, num_threads);
    for (auto& thread : threads) {
        thread.join();
    }
    progress_thread.join();

    cout<<"Output..."<<endl;
    ofstream recovery("../RecoveryHistory/"+recovFN+"_Recovery.txt");
    recovery<<fixed<<setprecision(10);
    for(int i=0;i<m;++i)recovery<<recovStream[i].str();
    recovery.close();

    ofstream full("../RecoveryHistory/"+recovFN+"_Full.txt");
    full<<m<<'\n';
    for(int i=0;i<m;++i)full<<matchStream[i].str();
    full.close();
    return 0;
}

void ReadVectors(){
    cout<<"Reading Embedded Road Vectors..."<<endl;
    ifstream file(ROADVECTOR);
    int num_roads;
    file >> num_roads;
    for (int i = 0; i <= min(num_roads,PATH_NUM-1); ++i) {
        int road_id;
        file >> road_id;
        std::vector<float> vector(vec_len);
        for (int j = 0; j < vec_len; ++j) file >> vector[j];
        road_vectors[road_id]=vector;
    }
    file.close();
}
