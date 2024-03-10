#ifndef MAKE_DATA_TRAFFIC_H
#define MAKE_DATA_TRAFFIC_H

#define PATH_NUM 65536
#include <unordered_map>
class TrafficHandler{
private:
    struct Last{
        int trajid,from,to,stamp,elapsed;
        double dist;
    };
    struct Traffic{
        short score[86402]{};
        //short cyclePred[24]{};
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
public:
    void init(const char* filename);
    double query(int roadID, int toID, long long timestamp) const;
};

#endif //MAKE_DATA_TRAFFIC_H
