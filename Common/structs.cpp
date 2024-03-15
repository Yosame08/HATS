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

unsigned BitInt::chk(unsigned x) {
    return v[x>>5] & (1<<(x&0x1F));
}

void BitInt::set(unsigned x) {
    v[x>>5] |= 1<<(x&0x1F);
}