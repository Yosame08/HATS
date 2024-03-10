#ifndef STRUCTS_H
#define STRUCTS_H

#include <vector>

struct G{
    std::vector<int>node[1048576];
    void connect(int nodeID, int roadID){
        node[nodeID].push_back(roadID);
    }
};

struct Point;
struct Vector {
    double x, y;
    Vector(double a,double b):x(a),y(b){}
    explicit Vector(const Point &p);
    friend Vector operator-(const Vector &a, const Vector &b) {
        return Vector{(a.x) - (b.x), (a.y) - (b.y)};
    }
    friend Vector operator*(const Vector &a, const double &b) {
        return Vector{(a.x) * b, (a.y) * b};
    }
    friend double operator%(const Vector a, const Vector b) {
        return (a.x) * (b.y) - (b.x) * (a.y);
    }//Cross Product
    friend double operator*(const Vector a, const Vector b) {
        return (a.x) * (b.x) + (a.y) * (b.y);
    }//Dot Product
    [[nodiscard]] double length() const;
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

struct PointLL{
    double lat,lon;
    PointLL() = default;
    PointLL(double a, double b): lat(a), lon(b){}
    /// Returns the great circle distance between two points (PointLL)
    /// \param x Another point (PointLL)
    /// \return Great circle distance
    [[nodiscard]] double dist(PointLL x) const;
    friend Vector operator-(const PointLL &a, const PointLL &b) {
        return Vector{(a.lat) - (b.lat), (a.lon) - (b.lon)};
    }
    friend PointLL operator+(const PointLL &a, const Vector &b) {
        return PointLL{(a.lat) + (b.x), (a.lon) + (b.y)};
    }
    friend PointLL operator+(const PointLL &a, const PointLL &b) {
        return PointLL{(a.lat) + (b.lat), (a.lon) + (b.lon)};
    }
    friend PointLL operator/(const PointLL a, const int b) {
        return PointLL{(a.lat)/b, (a.lon)/b};
    }
    friend bool operator==(const PointLL &a, const PointLL &b) {
        return a.lat==b.lat&&a.lon==b.lon;
    }
};

struct Trace {
    PointLL p;
    long long timestamp;
//    explicit operator const Point&() const {
//        return p;
//    }
};

struct Line{
    PointLL startLL,endLL; // lat/lon
    float len;
    explicit Line(const PointLL &s,const PointLL &e){
        startLL=s,endLL=e;
        len = startLL.dist(endLL);
    }
};

struct Segments{
    Line line;
    float sumPrev,sumAfter;
};

struct Road{
    int level, from, to;
    std::vector<Segments>seg;
    std::vector<float>angle;
};

struct PathNode{
    int roadID;
    long long timestamp;
    float toNodeDist, vel;
};
using Path = std::vector<PathNode>;

struct SearchNode{
    double prob;
    float toNodeDist;
    int roadID,prev,pointID;
    Path path;
};

struct GridInfo{
    int roadID, segID;
};

struct Candidate{
    int roadID;
    float toNodeDist;
    double prob;
};

struct SearchRes{
    double prob;
    float length, angle;
    int node;
    Path path;
};

struct QueueInfo{
    int level, node;
    double accuProb;
    float len, angle;
    bool operator <(const QueueInfo &b) const{return accuProb < b.accuProb;}
};

struct QInfo{
    int level, node;
    float len, toDstLen, angle;
    bool operator <(const QInfo &b) const{return len+toDstLen > b.len+b.toDstLen;}
};
#endif //STRUCTS_H
