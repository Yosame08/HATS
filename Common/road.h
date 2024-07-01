//
// Created by yosame on 24-6-15.
//

#ifndef INC_4_SOLVE_ROAD_H
#define INC_4_SOLVE_ROAD_H

#include "structs.h"
#include <vector>

struct Segment {
    Line line;
    double sumPrev, sumAfter;
};

class Road {
public:
    int level, from, to;
    std::vector<Segment> seg;
    std::vector<double> angle;
    [[nodiscard]] double length() const;
};


#endif//INC_4_SOLVE_ROAD_H
