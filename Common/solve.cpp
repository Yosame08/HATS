//
// Created by yosame on 24-6-15.
//

#include "solve.h"
#include "definitions.h"
#include "safeIO.h"
#include <atomic>
#include <fstream>
#include <sstream>
using namespace std;
typedef unsigned long long ull;
atomic<int> failed;

bool Solve::solveAssert(int id, bool condition, const std::string &cause) {
    if (condition) return true;
    recovery[id] << "Failed\n"
                 << -id - 1 << '\n';
    string out = (mode == Preprocess ? "Info: In path " : "Error: In path ") + to_string(id) + ": " + cause;
    safeCERR(out);
    ++failed;
    return false;
}

void Solve::initRoadNet(const std::string &edgeFile, const std::string &typeFile) {
    graph.loadEdgeOSM(edgeFile);
    graph.loadTypeOSM(typeFile);
}

void Solve::emptyTraffic() {
    for(double & i : roadSpeed)i=1;
}

void Solve::initModel(const string &vectorFile, const string &modelFile) {
    predictor.readRoadVectors(vectorFile);
    predictor.loadModel(modelFile);
}

void Solve::readTraces(const std::string &traceFile, bool fourPerRow, bool fixedInterval) {
    safeCOUT("Reading traces from " + traceFile);
    ifstream tr(traceFile);
    if(!tr)throw runtime_error("Cannot open file " + traceFile);
    long long repeat = 1;
    traces.reserve(TraceNumber);
    for (int m = 0; !tr.eof(); ++m) {
        traces.emplace_back();
        if(traces.size()>10000000)throw runtime_error("Error in reading traces, maybe trouble encountered when reading test input");
        if (!(m & 1023)) safeCOUTRaw("\rRead " + to_string(m >> 10) + "K trajectories");
        bool ok = true;
        while (true) {
            long long stamp, roadID;
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
            tr >> lat >> lon;
            if (fourPerRow) tr >> roadID;
            if (fixedInterval && !traces[m].empty() && abs(stamp - traces[m].back().timestamp) != RecoverInterval) {
                safeCERR("In trace " + to_string(m) + ", interval is not " + to_string(RecoverInterval) + ", ignored");
                std::this_thread::sleep_for(std::chrono::milliseconds (300));
                ok = false;
            }
            if (ok) traces[m].push_back({{lat, lon}, stamp});
        }
        if (!ok) --m;
    }
    while((!traces.empty())&&traces.back().empty())traces.pop_back();
}

Searched Solve::findPath(const Status &old, const Candidate &now, const GPS &last, const GPS &curr, double minProb) {
    const int pruneA = (mode == Preprocess) ? 1 : 2, pruneB = (mode == Preprocess ? 1 : 3);
    const int &fromRoad = old.roadID, &toRoad = now.roadID, span = static_cast<int>(curr.timestamp - last.timestamp);
    const double &toNodeDistA = old.toNodeDist, fromNodeDistB = graph.lengthOf(toRoad) - now.toNodeDist;
    const double greatCircle = last.p.dist(curr.p);
    vector<Searching> search;
    search.push_back({PathNode{fromRoad, toNodeDistA, (double)last.timestamp}, -1, 0, toNodeDistA,
                      graph.TurnedAngle(fromRoad, 0) - graph.TurnedAngle(fromRoad, toNodeDistA), toNodeDistA / speedOf(fromRoad)});
    BitInt vis;
    priority_queue<pair<double, int>> pq;
    pq.emplace(1, 0);
    while (!pq.empty()) {
        int lastNode = pq.top().second;
        const auto &lastInfo = search[lastNode];
        int atRoad = lastInfo.pNode.roadID;
        if (atRoad == toRoad && search.size() != 1) {
            // This is the road where the destination is located
            Searched result{pq.top().first, lastInfo.len, lastInfo.angle, make_unique<Path>()};
            stack<int> path;
            for (int i = lastNode; i != -1; i = search[i].prev) path.push(i);
            while (!path.empty()) {
                result.path->push_back(search[path.top()].pNode);
                path.pop();
            }
            result.path->push_back({toRoad, now.toNodeDist, (double)curr.timestamp});
            return result;
        }
        pq.pop();
        if (vis.chk(atRoad)) continue;
        vis.set(atRoad);
        int node = graph.roadTo(atRoad);
        for (const auto &to: graph.node[node]) {
            if (vis.chk(to)) continue;
            double angle = lastInfo.angle;
            if (graph.lengthOf(to) >= DegMinLen) {
                int angleNode = lastNode;
                for (; angleNode >= 0 && graph.lengthOf(search[angleNode].pNode.roadID) < DegMinLen; angleNode = search[angleNode].prev)
                    ;
                if (angleNode >= 0) {
                    if (angleNode == lastNode) angle += graph.GetTurnAngle(atRoad, to);
                    else
                        angle += graph.DiscontinuousAngle(search[angleNode].pNode.roadID, to);
                }
            }
            double newPassed = (to == toRoad) ? fromNodeDistB : graph.lengthOf(to);
            double leftGreatCircle = (to == toRoad ? 0 : graph.endOf(to).dist(curr.p));
            double totLen = lastInfo.len + newPassed;
            if (totLen + leftGreatCircle >= span * SpeedLimit) continue;
            double consume = lastInfo.estTime + newPassed / speedOf(graph.levelOf(to));
            angle += graph.TurnedAngle(to, graph.lengthOf(to) - newPassed);
            if (to != toRoad) {// Destination
                if (lastInfo.seqID * pruneA > span) continue;
                if (lastInfo.seqID * pruneB > span && greatCircle < leftGreatCircle) continue;
            }
            double outProb = criterion->Prob(totLen, leftGreatCircle, greatCircle, angle, consume, span, true);
            if (outProb < minProb) continue;
            pq.emplace(outProb, static_cast<int>(search.size()));
            search.push_back({PathNode{to, graph.lengthOf(to), lastInfo.pNode.timestamp}, lastNode, lastInfo.seqID + 1, totLen, angle, consume});
        }
    }
    return {-1};
}

void Solve::multiSolve(int threads) {
    ThreadPool pool(threads);
//    traces.resize(500);
    pool.startProgressDisplay(traceCnt());
//    for (int i = 0; i < traceCnt(); ++i) {
    for (int i = 8; i <= 8; ++i) {
        pool.enqueue([i, this] { pathSearch(i); });
    }
}

void Solve::pathSearch(int id) {
    const int range = (mode == Preprocess ? 60 : 80);
    const auto &trace = traces[id];
    vector<Candidate> founds;
    graph.Point2RoadUnique(range, trace[0].p, founds);
    if (!solveAssert(id, !founds.empty(), "No road near point 0 founds")) return;
    vector<Status> statuses;
    statuses.reserve(founds.size() * trace.size());
    for (const auto &f: founds) {
        statuses.push_back({criterion->PtMatchProb(f.errDist), f.toNodeDist, 0, f.roadID, -1, make_unique<Path>(Path{{f.roadID, f.toNodeDist, (double)trace[0].timestamp}}), 0, f.errDist});
    }
    int lastFrom = 0, lastTo = static_cast<int>(statuses.size()) - 1;
    unordered_set<ull> noPath;
    for (int i = 1; i < trace.size(); ++i) {
        if (!solveAssert(id, trace[i].timestamp - trace[i - 1].timestamp < 4096, "Too long interval of trajectory")) return;
        founds.clear();
        graph.Point2RoadUnique(range, trace[i].p, founds);
        if (!solveAssert(id, !founds.empty(), "No road near point " + to_string(i) + " founds")) return;
        for (auto &now: founds) {
            Status maxState{-1, -1, -1, -1, -1, nullptr};
            for (int l = lastFrom; l <= lastTo; ++l) {
                const Status &old = statuses[l];
                ull hashed = now.roadID | (static_cast<ull>(old.roadID) << 32);
                if (noPath.count(hashed)) continue;
                if (old.roadID == now.roadID) {
                    double ground = old.toNodeDist - now.toNodeDist;
                    double angle = graph.TurnedAngle(now.roadID, now.toNodeDist) - graph.TurnedAngle(old.roadID, old.toNodeDist);
                    int interval = static_cast<int>(trace[i].timestamp - trace[i - 1].timestamp);
                    double allProb = old.prob * criterion->PtMatchProb(now.errDist) * criterion->Prob(ground, 0, trace[i].p.dist(trace[i - 1].p), angle, ground / speedOf(now.roadID), interval, false);
                    if (allProb >= maxState.prob) {
                        maxState = {allProb, now.toNodeDist, ground, now.roadID, l, make_unique<Path>(), angle, now.errDist};
                        maxState.path->push_back({old.roadID, old.toNodeDist, (double)trace[i - 1].timestamp});
                        maxState.path->push_back({now.roadID, now.toNodeDist, (double)trace[i].timestamp});
                    }
                } else {
                    double stateProb = old.prob * criterion->PtMatchProb(now.errDist);
                    if (stateProb <= maxState.prob) continue;
                    Searched result = findPath(old, now, trace[i - 1], trace[i], stateProb == 0 ? 0 : maxState.prob / stateProb);
                    if (result.prob == -1) {
                        noPath.insert(hashed);
                        continue;
                    }
                    double allProb = stateProb * result.prob;
                    if (allProb >= maxState.prob) maxState = {allProb, now.toNodeDist, result.length, now.roadID, l, std::move(result.path), result.angle, now.errDist};
                }
            }
            assert(!isnan(maxState.prob) && !isinf(maxState.prob));
            if (maxState.prob != -1) statuses.push_back(std::move(maxState));
        }
        if (statuses.size() == lastTo + 1) {
            string info = "HMM break at point " + to_string(i);
            if (mode == Preprocess){
                solveAssert(id, false, info);
                return;
            }
            else
                safeCERR(info);
            int maxPt;
            double maxProb = -1;
            for (int k = lastFrom; k <= lastTo; ++k) {
                if (statuses[k].prob > maxProb) {
                    maxProb = statuses[k].prob;
                    maxPt = k;
                }
            }
            for (auto &now: founds) statuses.push_back({criterion->PtMatchProb(now.errDist), now.toNodeDist, 0, now.roadID, maxPt, make_unique<Path>(Path{{now.roadID, now.toNodeDist, (double)trace[i].timestamp}}), 0, now.errDist});
        }
        lastFrom = lastTo + 1, lastTo = static_cast<int>(statuses.size()) - 1;
        double maxProb = -1;
        for (int k = lastFrom; k <= lastTo; ++k)
            if (statuses[k].prob > maxProb) maxProb = statuses[k].prob;
        if (maxProb == 0) {
            safeCERR("Warning: Max probability score is 0 at trace " +to_string(id)+ " point " + to_string(i));
            for (int k = lastFrom; k <= lastTo; ++k) statuses[k].prob = 1;
        } else
            for (int k = lastFrom; k <= lastTo; ++k) statuses[k].prob /= maxProb;
    }
    double maxProb = -1;
    int outID;
    for (int k = lastFrom; k <= lastTo; ++k)
        if (statuses[k].prob >= maxProb) maxProb = statuses[k].prob, outID = k;
    stack<int> path;
    for (int i = outID; i != -1; i = statuses[i].prev) path.push(i);
    vector<Status> finalStatuses;
    while (!path.empty()) {
        finalStatuses.push_back(std::move(statuses[path.top()]));
        path.pop();
    }
    assert(finalStatuses.size() == trace.size());
    switch (mode) {
        case Preprocess:
            linearInterpolation(finalStatuses, id);
            break;
        case Final:
            modelInterpolation(finalStatuses, id);
            break;
    }
}

vector<pair<float, double>> Solve::velSequence(const Path &path, long long timeOld, long long timeNow, double journey, double fullLength, int nxtID) {
    auto timestamp = static_cast<double>(timeOld);
    vector<pair<float, double>> result;// velocity, timestamp
    auto predToNxt = [&](double pass, int fromID, int toID, double toNode) {
        if (pass <= 0) return;
        if(fromID == toID) toID = nxtID;
        while (true) {
            float vel = predictor.velPredict(fromID, toID, static_cast<float>(toNode),
                                             static_cast<long long>(timestamp), static_cast<float>(journey / fullLength));
            if (vel > pass) {
                timestamp += pass / vel;
                journey += pass;
                result.emplace_back(pass, pass / vel);
                break;
            } else {
                pass -= vel, toNode -= vel;
                timestamp += 1;
                journey += vel;
                result.emplace_back(vel, 1);
            }
        }
    };
    assert(path.at(path.size() - 2).roadID == path.back().roadID);
    for (int j = 1; j < path.size() - 1; ++j) {
        double toNodeDist = path[j - 1].toNodeDist;
        predToNxt(toNodeDist, path[j - 1].roadID, path[j].roadID, toNodeDist);
    }
    double needPass = path.at(path.size() - 2).toNodeDist - path.back().toNodeDist;
    predToNxt(needPass, path.back().roadID, path.back().roadID, path.at(path.size() - 2).toNodeDist);
    if (result.empty()) return {make_pair(fullLength, timeNow - timeOld)};
    else {
        double scale = static_cast<double>(timeNow - timeOld) / (timestamp - static_cast<double>(timeOld));
        if(isinf(scale)) return {make_pair(fullLength, timeNow - timeOld)};
        for (auto &x: result) x.second *= scale;
        return result;
    }
}

void Solve::modelInterpolation(std::vector<Status> &statuses, int id) {
    const auto &trace = traces[id];
    Path result;
    vector<int> fullPath;
    assert(statuses.begin()->path->size() == 1);
    result.push_back(statuses.begin()->path->front());
    double journey = 0, fullLength = 0;
    fullPath.push_back(statuses[0].path->begin()->roadID);
    for (int i = 1; i < statuses.size(); ++i) fullLength += statuses[i].length;
    for (int i = 1; i < statuses.size(); ++i) {
        const Path &path = *statuses[i].path;
        if (!solveAssert(id, path.size() >= 2, "\n(Bug) Exists a node with no path\n")) return;
        if (!solveAssert(id,
                         path.begin()->roadID == result.back().roadID && path.begin()->toNodeDist == result.back().toNodeDist,
                         "\n(Bug) Inconsistency of neighboring statistical entries\n")) return;
        int nxtID = statuses[i].path->back().roadID;
        for(int j = i + 1; j < statuses.size() && nxtID == statuses[i].path->front().roadID; ++j) nxtID = statuses[j].path->back().roadID;
        auto predVel = velSequence(path, trace[i - 1].timestamp, trace[i].timestamp, journey, fullLength, nxtID);
        double timeAccum = 0, distAccum = 0;
        auto it = path.begin();
        long long timestamp = trace[i - 1].timestamp;
        for (auto &x: predVel) {
            while (x.second + timeAccum > RecoverInterval) {
                double ratio = (RecoverInterval - timeAccum) / x.second;
                double newDistAccum = distAccum + ratio * x.first;
                auto newIt = it;
                while (newIt != path.end() && newDistAccum >= newIt->toNodeDist) {
                    newDistAccum -= newIt->toNodeDist;
                    ++newIt;
                }
                if (newIt == path.end()) --newIt;
                timestamp += RecoverInterval;
                result.push_back({newIt->roadID, newIt->toNodeDist - newDistAccum, (double)timestamp});
                timeAccum -= RecoverInterval;
            }
            timeAccum += x.second;
            distAccum += x.first;
        }
        if (result.back().timestamp == trace[i].timestamp) result.pop_back();
        result.emplace_back(path.back());
        for (int j = 1; j < path.size(); ++j) {
            int now = path.at(j).roadID;
            if (fullPath.back() != now) fullPath.push_back(now);
        }
    }
    recovery[id] << fixed << setprecision(8);
    for (auto &p: result) {
        PtEarth pt = graph.Road2Point(p.roadID, p.toNodeDist);
        recovery[id] << static_cast<long long>(p.timestamp) << ' ' << pt.lat << ' ' << pt.lon << ' ' << p.roadID << '\n';
    }
    recovery[id] << -id - 1 << '\n';
    for (auto &i: fullPath) matching[id] << i << ' ';
    matching[id] << '\n';
}

void Solve::linearInterpolation(vector<Status> &statuses, int id) {
    const auto &trace = traces[id];
    Path result;
    vector<int> fullPath;
    fullPath.push_back(statuses[0].path->begin()->roadID);
    assert(statuses.begin()->path->size() == 1);

    gpsErrors[id].push_back(statuses[0].error);
    for (int i = 1; i < statuses.size(); ++i) {
        const Path &path = *statuses[i].path;
        const double span = path.back().timestamp - path.front().timestamp;
        double totLen = 0;
        result.push_back(path.front());
        for (int j = 1; j < path.size() - 1; ++j){
            if(path[j-1].roadID == path[j].roadID) totLen += path[j-1].toNodeDist - path[j].toNodeDist;
            else totLen += path[j-1].toNodeDist;
            result.push_back({path[j].roadID, path[j].toNodeDist, path[0].timestamp + (statuses[i].length == 0 ? 0 : (totLen / statuses[i].length)) * span});
            if(path[j].roadID != fullPath.back())fullPath.push_back(path[j].roadID);
        }
        double accuAngle=0, accuDist=0;
        for(int pre=1;pre<=min(i,MaxMissing);++pre){
            accuAngle += statuses[i-pre+1].angle;
            accuDist += statuses[i-pre+1].length;
            turnAngles[id][pre].push_back(accuAngle);
            difDists[id][pre].push_back(accuDist - trace[i].p.dist(trace[i-pre].p));
        }
        passLen[graph.levelOf(statuses[i].roadID)] += statuses[i].length;
        passTime[graph.levelOf(statuses[i].roadID)] += span;
        gpsErrors[id].push_back(statuses[i].error);
    }

    int ptr = 0;
    assert(result[0].roadID == fullPath.front());
    for (int i = 0; i < result.size() - 1; ++i){
        if(result[i+1].timestamp == result[i].timestamp)continue;
        if(result[i].roadID == fullPath[ptr] && ptr != fullPath.size() - 1) ++ptr;
        preprocess[id] << id << ',' << result[i].roadID << ',' << fullPath[ptr] << ',' << ((long long)result[i].timestamp % 86400) + (result[i].timestamp - (long long)result[i].timestamp) <<
                ',' << result[i].toNodeDist << ',' << result[i+1].timestamp - result[i].timestamp << '\n';
    }
    preprocess[id] << id << ',' << result.back().roadID << ',' << fullPath.back() << ',' << ((long long)result.back().timestamp % 86400) + (result.back().timestamp - (long long)result.back().timestamp) <<
            ',' << result.back().toNodeDist << ',' << 0 << '\n';
    for (auto &i: fullPath) matching[id] << i << ' ';
    matching[id] << '\n';
}

void Solve::outputRecovery(const string &recoveryFile) {
    safeCOUT("Output...");
    ofstream recoverStream("../../ResultLog/" + recoveryFile + "_recovery.txt");
    recoverStream << fixed << setprecision(8);
    for (int i = 0; i < traceCnt(); ++i) recoverStream << recovery[i].str();
    recoverStream.close();
}

void Solve::outputPreprocess(const string &preprocessFile) {
    ofstream pre("../../Intermediate/" + preprocessFile + "_preprocess.csv");
    pre << fixed << setprecision(2);
    pre << "trajID,originID,transitionID,timestamp,toNode,elapsed\n";
    for (int i = 0; i < traceCnt(); ++i) pre << preprocess[i].str();
    pre.close();
    if(preprocessFile == "train"){
        ofstream turnCount("../../Intermediate/train_turn.stat"), difDistCount("../../Intermediate/train_dist.stat");
        turnCount<<fixed<<setprecision(1);
        difDistCount<<fixed<<setprecision(1);
        for(int j=2;j<=MaxMissing;++j){
            turnCount<<j*RecoverInterval<<" Secs:\n";
            for(int i=0;i<traceCnt();++i)for(auto x:turnAngles[i][j])turnCount<<(x*180/M_PI)<<' ';
            turnCount<<'\n';
            difDistCount<<j*RecoverInterval<<" Secs:\n";
            for(int i=0;i<traceCnt();++i)for(auto x:difDists[i][j])difDistCount<<x<<' ';
            difDistCount<<'\n';
        }

        ofstream param("../../Intermediate/params");
        for(int i=0;i<7;++i){
            param<<"lv"<<i+1<<": "<<passLen[i]/passTime[i]<<'\n';
        }
        double avgGPSError = 0;
        size_t gpsNums=0;
        for(const auto &x:gpsErrors){
            for(const auto &i:x)avgGPSError+=i;
            gpsNums+=x.size();
        }
        avgGPSError /= gpsNums;
        //solve std error
        double var = 0;
        for(const auto &x:gpsErrors){
            for(const auto &i:x)var += (i - avgGPSError) * (i - avgGPSError);
        }
        var /= gpsNums;
        param<<"gps_std_error: "<<sqrt(var)<<'\n';
    }
}

void Solve::outputMatching(const string &matchingFile) {
    string folder = (mode == Preprocess) ? "Intermediate" : "ResultLog";
    ofstream full("../../" + folder + '/' + matchingFile + "_full.txt");
    full << traceCnt() << '\n';
    for (int i = 0; i < traceCnt(); ++i) full << matching[i].str();
    full.close();
}

int Solve::traceCnt() {
    return static_cast<int>(traces.size());
}

double Solve::speedOf(int roadID) {
    return roadSpeed[graph.levelOf(roadID)];
}

void Solve::initTraffic(const string &roadSpeedFile) {
    ifstream speed(roadSpeedFile);
    string in;
    for (double & i : roadSpeed) speed >> in >> i;
}

void Solve::initLight(const string &traceFile) {
    light.parse(traceFile);
}
