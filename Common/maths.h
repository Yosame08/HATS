#ifndef MYMATH_H
#define MYMATH_H

struct Point;
struct PointLL;
struct Line;
struct Vector;

// 定义地球半径（单位：千米）
const double EARTH_RADIUS = 6371.0;
// 定义地图原点（地球转平面直角坐标系的原点）
// const double lat_origin = 31, lon_origin = 121;
// const double lat_origin = 41.15, lon_origin = -8.61;

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
