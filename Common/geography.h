//
// Created by yosame on 24-6-15.
//

#ifndef INC_4_SOLVE_GEOGRAPHY_H
#define INC_4_SOLVE_GEOGRAPHY_H

#include "structs.h"

double greatCircle(double lat1, double lon1, double lat2, double lon2);
Point latLonToXY(const PtEarth &p, const PtEarth &origin);
double Angle(const Vector &a, const Vector &b);
double DistPointSeg(const Line &seg, const PtEarth &measureLL, PtEarth &crossLL);

#endif//INC_4_SOLVE_GEOGRAPHY_H
