//
// Created by yosame on 24-6-15.
//

#include "geography.h"
#include <cmath>

const double EARTH_RADIUS = 6371.0;

double deg2rad(double deg) {
    return (deg * M_PI / 180);
}

// param: latitude and longitude of two points
double greatCircle(double lat1, double lon1, double lat2, double lon2) {
    // 将经纬度转换为弧度
    lat1 = deg2rad(lat1);
    lat2 = deg2rad(lat2);

    // 使用Haversine公式计算大圆距离
    double dlon = deg2rad(lon2 - lon1);
    double dlat = lat2 - lat1;
    double a = pow(sin(dlat / 2), 2) + cos(lat1) * cos(lat2) * pow(sin(dlon / 2), 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));

    // 返回大圆距离（单位：米）
    return EARTH_RADIUS * c * 1000;
}

// 将经纬度转换为平面直角坐标
Point latLonToXY(const PtEarth &p, const PtEarth &origin) {
    const double &lat_origin = origin.lat, &lon_origin = origin.lon;
    // 计算两点之间的大圆距离
    double distance = greatCircle(lat_origin, lon_origin, p.lat, p.lon);

    // 计算两点之间的方位角
    double bearing = atan2(sin(p.lon - lon_origin) * cos(p.lat), cos(lat_origin) * sin(p.lat) - sin(lat_origin) * cos(p.lat) * cos(p.lon - lon_origin));

    // 将大圆距离和方位角转换为平面直角坐标
    double x = distance * cos(bearing);
    double y = distance * sin(bearing);

    return {x, y};
}
double Square(double x) { return x * x; }
double PDistance(const Point &a, const Point &b) { return sqrt(Square(a.y - b.y) + Square(a.x - b.x)); }
double PSquareDist(const Point &a, const Point &b) { return Square(a.y - b.y) + Square(a.x - b.x); }

/// calculate the distance between a point and a segment
/// @param seg the segment
/// @param measureLL the point
/// @param crossLL out: the point on the segment that is closest to the measureLL
/// @return the distance between the point and the segment
double DistPointSeg(const Line &seg, const PtEarth &measureLL, PtEarth &crossLL) {
    PtEarth origin = (seg.startLL + seg.endLL) / 2;
    Point starts = latLonToXY(seg.startLL, origin), ends = latLonToXY(seg.endLL, origin), measure = latLonToXY(measureLL, origin);
    Vector vector = ends - starts;
    Vector perpend = {-vector.y, vector.x};//Rotate 90 degree 垂直向量
    // Get the intersection
    double t = ((measure - starts) % perpend) / (vector % perpend);
    Point cross = starts + vector * t;
    crossLL = seg.startLL + (seg.endLL - seg.startLL) * t;
    // 交点在线段上（两端点形成的矩形内部）
    double lowx, highx, lowy, highy;
    if (vector.x < 0) lowx = ends.x, highx = starts.x;
    else
        lowx = starts.x, highx = ends.x;
    if (vector.y < 0) lowy = ends.y, highy = starts.y;
    else
        lowy = starts.y, highy = ends.y;
    if (lowx <= cross.x && cross.x <= highx && lowy <= cross.y && cross.y <= highy) return PDistance(measure, cross);
    // 交点不在线段上
    else {
        double left = PSquareDist(measure, starts), right = PSquareDist(measure, ends);
        if (left < right) {
            crossLL = seg.startLL;
            return sqrt(left);
        } else {
            crossLL = seg.endLL;
            return sqrt(right);
        }
    }
}

double Angle(const Vector &a, const Vector &b) {
    double cosVal = (a * b) / (a.length() * b.length());
    if (cosVal > 1) cosVal = 1;
    else if (cosVal < -1)
        cosVal = -1;
    return std::acos(cosVal);
}