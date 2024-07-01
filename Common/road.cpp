//
// Created by yosame on 24-6-15.
//

#include "road.h"
double Road::length() const {
    return seg.back().sumPrev;
}
