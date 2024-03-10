#ifndef STRUCTS_H
#define STRUCTS_H

#include "mymath.h"
#include <vector>
//#define (a.x) (a.x)
//#define (b.x) (b.x)
//#define (a.y) (a.y)
//#define (b.y) (b.y)
struct Vector {
    double x, y;
    friend Vector operator-(const Vector &a, const Vector &b) {
        return Vector{(a.x) - (b.x), (a.y) - (b.y)};
    }
    friend Vector operator*(const Vector &a, const double &b) {
        return Vector{(a.x) * b, (a.y) * b};
    }
    friend double operator%(const Vector a, const Vector b) {
        return (a.x) * (b.y) - (b.x) * (a.y);
    }//Cross Product
};

struct Point {
    double x, y;
    Point() = default;
    Point(double a, double b) : x(a), y(b) {}
    friend Vector operator-(const Point &a, const Point &b) {
        return Vector{(a.x) - (b.x), (a.y) - (b.y)};
    }
    friend Point operator+(const Point &a, const Vector &b) {
        return Point{(a.x) + (b.x), (a.y) + (b.y)};
    }
    friend Point operator+(const Point &a, const Point &b) {
        return Point{(a.x) + (b.x), (a.y) + (b.y)};
    }
    friend Point operator/(const Point a, const int b) {
        return Point{(a.x)/b, (a.y)/b};
    }
    friend bool operator==(const Point &a, const Point &b) {
        return a.x==b.x&&a.y==b.y;
    }
};

struct Trace {
    Point p;
    long long timestamp;
    double orilat,orilon;
    explicit operator const Point&() const {
        return p;
    }
};

struct Line{
    Point start,end;
    Point startLL,endLL; // lat/lon
    double len, dire;
    explicit Line(const Point &s,const Point &e,const Point &sLL,const Point &eLL){
        start=s,end=e;
        len = Distance(start,end);
        dire = Angle(start,end);
        startLL=sLL,endLL=eLL;
    }
};

struct Segments{
    Line line;
    double sumPrev,sumAfter;
};

struct Road{
    int level, from, to;
    std::vector<Segments>seg;
};

struct PathNode{
    int roadID;
    long long timestamp;
    double toNodeDist;
};
using Path = std::vector<PathNode>;

struct SearchNode{
    double prob,toNodeDist;
    int roadID,prev,pointID;
    Path path;
};

struct GridInfo{
    int roadID, segID;
};

struct Candidate{
    int roadID;
    double toNodeDist, prob;
};

struct SearchRes{
    double prob, length;
    int node;
    Path path;
};

struct QInfo{
    int level, node;
    double len, toDstLen;
    bool operator <(const QInfo &b) const{return len+toDstLen > b.len+b.toDstLen;}
};
#endif //STRUCTS_H
