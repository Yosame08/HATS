//
// Created by yosame on 24-6-15.
//

#ifndef INC_4_SOLVE_STRUCTS_H
#define INC_4_SOLVE_STRUCTS_H

#include "definitions.h"
#include <atomic>
#include <condition_variable>
#include <cstdlib>
#include <functional>
#include <mutex>
#include <queue>
#include <thread>

enum SolveMode{
    Preprocess, Final
};

struct Point;
struct Vector {
    double x, y;
    Vector(double a, double b) : x(a), y(b) {}
    explicit Vector(const Point &p);
    friend Vector operator-(const Vector &a, const Vector &b) {
        return Vector{(a.x) - (b.x), (a.y) - (b.y)};
    }
    friend Vector operator*(const Vector &a, const double &b) {
        return Vector{(a.x) * b, (a.y) * b};
    }
    friend double operator%(const Vector a, const Vector b) {
        return (a.x) * (b.y) - (b.x) * (a.y);
    }// Cross Product
    friend double operator*(const Vector a, const Vector b) {
        return (a.x) * (b.x) + (a.y) * (b.y);
    }// Dot Product
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
        return Point{(a.x) / b, (a.y) / b};
    }
    friend bool operator==(const Point &a, const Point &b) {
        return a.x == b.x && a.y == b.y;
    }
};

struct PtEarth {
    double lat, lon;
    PtEarth() = default;
    PtEarth(double a, double b) : lat(a), lon(b) {}
    /// Returns the great circle distance between two points (PtEarth)
    /// \param x Another point (PtEarth)
    /// \return Great circle distance
    [[nodiscard]] double dist(PtEarth x) const;
    friend Vector operator-(const PtEarth &a, const PtEarth &b) {
        return Vector{(a.lat) - (b.lat), (a.lon) - (b.lon)};
    }
    friend PtEarth operator+(const PtEarth &a, const Vector &b) {
        return PtEarth{(a.lat) + (b.x), (a.lon) + (b.y)};
    }
    friend PtEarth operator+(const PtEarth &a, const PtEarth &b) {
        return PtEarth{(a.lat) + (b.lat), (a.lon) + (b.lon)};
    }
    friend PtEarth operator/(const PtEarth a, const int b) {
        return PtEarth{(a.lat) / b, (a.lon) / b};
    }
    friend bool operator==(const PtEarth &a, const PtEarth &b) {
        return a.lat == b.lat && a.lon == b.lon;
    }
};

struct Line {
    PtEarth startLL, endLL;// lat/lon
    double len;
    explicit Line(const PtEarth &s, const PtEarth &e) : startLL(s), endLL(e) {
        len = startLL.dist(endLL);
    }
};

struct BitInt {
    unsigned v[(RoadNumber >> 5) + 1]{};
    unsigned chk(unsigned x);
    void set(unsigned x);
};

struct stdPairHash {
    template<class T1, class T2>
    std::size_t operator()(const std::pair<T1, T2> &pair) const {
        return (std::hash<T1>()(pair.first) << 16) ^ std::hash<T2>()(pair.second);
    }
};

struct GridInfo {
    int roadID, segID;
};
using RoadGrid = std::unordered_map<std::pair<int, int>, std::vector<GridInfo>, stdPairHash>;

struct PathNode {
    int roadID;
    double toNodeDist;
    double timestamp;
};
using Path = std::vector<PathNode>;

struct Searching {
    PathNode pNode;
    int prev, seqID;
    double len, angle, estTime;
};

struct Status {
    double prob, toNodeDist, length;
    int roadID, prev;
    std::unique_ptr<Path> path;
    double angle, error;
};

struct Candidate {
    int roadID;
    double toNodeDist;
    double errDist;
};

struct Searched {
    double prob, length, angle;
    std::unique_ptr<Path> path;
};

struct GPS {
    PtEarth p;
    long long timestamp;
};
using Trace = std::vector<GPS>;
using Traces = std::vector<Trace>;

class ThreadPool {
public:
    explicit ThreadPool(size_t threads);
    template<class F>
    void enqueue(F &&f) {
        {
            std::unique_lock<std::mutex> lock(queue_mutex);
            if (stop) throw std::runtime_error("enqueue on stopped ThreadPool");
            tasks.emplace(std::forward<F>(f));
        }
        condition.notify_one();
    }
    ~ThreadPool();
    void startProgressDisplay(int lim);
    void stopProgressDisplay();

private:
    std::vector<std::thread> workers;
    std::queue<std::function<void()>> tasks;
    std::mutex queue_mutex;
    std::condition_variable condition;
    bool stop;
    std::atomic<int> progress{0};
    std::atomic<bool> display_stop_requested{false};
    int limit;
    std::thread progress_thread;
};

#endif// INC_4_SOLVE_STRUCTS_H
