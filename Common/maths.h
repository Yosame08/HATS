#ifndef MYMATH_H
#define MYMATH_H

struct Point;
struct PointLL;
struct Line;
struct Vector;

const double EARTH_RADIUS = 6371.0;

double deg2rad(double);
double greatCircleDistance(double, double, double, double);
Point latLonToXY(const PointLL &p, const PointLL &origin);
double Square(double);
double PDistance(const Point&, const Point&);
double PSquareDist(const Point&, const Point&);
double DistPointSeg(const Line &seg, const PointLL &measure, PointLL &cross);
double Angle(const Vector &a, const Vector &b);
int distToTwo(int hour);
float CycleTime(long long stamp);

#endif //MYMATH_H
