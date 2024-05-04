#ifndef MAKE_DATA_TRAFFIC_H
#define MAKE_DATA_TRAFFIC_H

#include "definitions.h"
#include <unordered_map>
#include <bitset>
class TrafficHandler{
private:
    struct Last{
        int trajid,from,to,stamp,elapsed;
        double dist;
    };
    struct Traffic{
        short score[86402]{};
        std::bitset<86402> cross;
        int cnt=0;
        double percent=0;
        void stat();
    };
    enum Data{
        trajid,original,transition,hour,sec,distance,elapsed
    };
    std::unordered_map<int,Traffic*> lights[PATH_NUM+1];
    int lim{};
    void addInterval(int id, int to, int l, int r, short val);
    void init(const char* filename);
public:
    explicit TrafficHandler(const char* filename){
        init(filename);
    }
    double query(int roadID, int toID, long long timestamp, float toNodeDist) const;
};

#endif //MAKE_DATA_TRAFFIC_H
