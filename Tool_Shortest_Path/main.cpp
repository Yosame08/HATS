#include "structs.h"
#include "reads.h"
#include <vector>
#include <thread>
#include <sstream>
#include <fstream>
#include <iostream>
#include <atomic>
#include <cassert>
using namespace std;

G g;
Road roads[PATH_NUM];
vector<TraceRD>traces[524288];
ostringstream matchStream[524288];
GridType inGrid;

bool dijkstra(int fromID, int toID, deque<int> &out, long long span){
    BitInt vis;
    int prev[PATH_NUM];
    priority_queue<pair<float,int>>q;
    q.emplace(0,fromID);
    while(!q.empty()){
        int x=q.top().second;
        float len=-q.top().first;
        q.pop();
        if(vis.chk(x))continue;
        vis.set(x);
        for(auto y:g.node[roads[x].to]){
            if(vis.chk(y))continue;
            prev[y]=x;
            if(y==toID){
                while(x!=fromID){
                    out.push_front(x);
                    x=prev[x];
                }
                return true;
            }
            else{
                float dist = len + RoadLen(y);
                if(dist>span*40)continue;
                q.emplace(-dist,y);
            }
        }
    }
    return false;
}

std::atomic<int> progress(0);
void print_progress(int lim) {
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
        std::cout << "] (" << current << "/" << lim << ") " << std::flush;
        if (current >= lim) break;
        this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    cout << endl;
}

void process(int start, int end){
    for(int j=start;j<end;++j){
        const auto &traceNow = traces[j];
        vector<int>paths;
        paths.push_back(traceNow[0].roadID);
        for(int i=1;i<traceNow.size();++i){
            deque<int>path;
            if(dijkstra(paths.back(), traceNow[i].roadID, path, traceNow[i].tr.timestamp - traceNow[i-1].tr.timestamp)){
                while(!path.empty()){
                    paths.push_back(path.front());
                    path.pop_front();
                }
            }
            paths.push_back(traceNow[i].roadID);
        }
        ++progress;
        matchStream[j]<<paths[0]<<' ';
        int last = paths[0];
        for(int i=1;i<paths.size();++i){
            if(paths[i]==last)continue;
            auto &connect = g.node[roads[last].to];
            //clog<<i<<": "<<last<<"->"<<paths[i]<<endl;
            //assert(std::find(connect.begin(), connect.end(),paths[i]) != connect.end());
            matchStream[j]<<paths[i]<<' ';
            last = paths[i];
        }
    }
}

int main(){
    ios::sync_with_stdio(false);
    int m;
    ReadRoadNet(EDGEFILE,TYPEFILE,g,roads,inGrid);
    ReadTracesWithRoad("RN_recovery.txt", m, traces);

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

    ofstream out("Full_Matched_Dij.txt");
    out<<m<<'\n';
    for(int i=0;i<m;++i)out<<matchStream[i].str()<<'\n';
    return 0;
}