//
// Created by yosame on 24-6-15.
//

#ifndef INC_4_SOLVE_LIGHT_H
#define INC_4_SOLVE_LIGHT_H

#include <string>
#include <bitset>
#include <unordered_map>
#include "definitions.h"

class Light {
public:
    void parse(const std::string &traceFile);
    double query(int roadID, int toID, double timeDouble, double toNodeDist) const;
private:
    struct Traffic{
        char score[86400]{};
        std::bitset<86400> cross;
        int cnt=0;
        double percent=0;
        void stat();
        void modify(int l, int r, int val);
    };
    enum Data{
        trajID,origin,transition,timestamps,distance,elapsed
    };
    std::unordered_map<int,Traffic*> lights[RoadNumber];
    int lim{};
    void addInterval(int id, int to, double dl, double dr, int val, int mode);
};


#endif//INC_4_SOLVE_LIGHT_H
