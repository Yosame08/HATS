#include "maths.h"
#include "structs.h"
#include <cmath>

double PointLL::dist(PointLL x) const {
    return greatCircleDistance(lat,lon,x.lat,x.lon);
}

double Vector::length() const {
    return std::sqrt(x * x + y * y);
}

Vector::Vector(const Point &p):x(p.x),y(p.y){}

