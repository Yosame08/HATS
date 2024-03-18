#include "structs.h"
#include "reads.h"
#include <vector>
#include <thread>
#include <sstream>
#include <fstream>
#include <iostream>
#include <atomic>
using namespace std;

struct TraceRD{
    Trace tr;
    int roadID;
};
G g;
Road roads[PATH_NUM];
vector<TraceRD>traces[524288];
ostringstream matchStream[524288];
GridType inGrid;

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

void ReadTracesWithRoad(const std::string& traceFN, int&m, std::vector<TraceRD>traces[]){
    cout<<"Reading Traces..."<<endl;
    ifstream tr(traceFN);
    long long repeat=1;
    for(m=0;!tr.eof();++m){
        if(m&&m%1024==0)clog<<"\rRead "<<m/1024<<"K trajectories";
        bool ok=true;
        while(true){
            long long stamp;
            int roadID;
            double lat, lon;
            tr>>stamp;
            if(stamp<1){
                if(repeat == stamp){
                    --m;
                    break;
                }
                repeat = stamp;
                break;
            }
            tr>>lat>>lon>>roadID;
            if(!traces[m].empty()&&abs(stamp-traces[m].back().tr.timestamp)!=RECOVER_INTERVAL){
                cerr<<"In trace "<<m<<", interval is not "<<RECOVER_INTERVAL<<", ignore"<<endl;
                ok=false;
            }
            if(ok)traces[m].push_back({{{lat,lon},stamp},roadID});
        }
        if(!ok)--m;
    }
    clog<<'\n';
}

int main(){
    ios::sync_with_stdio(false);
    int m;
    ReadRoadNet(EDGEFILE,TYPEFILE,g,roads,inGrid);
    ReadTracesWithRoad("cmake-build-debug/output.txt", m, traces);
}