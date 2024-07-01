//
// Created by yosame on 24-6-15.
//

#include "structs.h"
#include "geography.h"
#include "safeIO.h"
#include <cmath>
using namespace std;

double PtEarth::dist(PtEarth x) const {
    return greatCircle(lat, lon, x.lat, x.lon);
}

double Vector::length() const {
    return std::sqrt(x * x + y * y);
}

Vector::Vector(const Point &p) : x(p.x), y(p.y) {
}

unsigned BitInt::chk(unsigned x) {
    return v[x >> 5] & (1 << (x & 0x1F));
}

void BitInt::set(unsigned x) {
    v[x >> 5] |= 1 << (x & 0x1F);
}

ThreadPool::ThreadPool(size_t threads) : stop(false), limit(0) {
    for (size_t i = 0; i < threads; ++i) {
        workers.emplace_back([this] {
            for (;;) {
                std::function<void()> task;
                {
                    std::unique_lock<std::mutex> lock(this->queue_mutex);
                    this->condition.wait(lock, [this] { return this->stop || !this->tasks.empty(); });
                    if (this->stop && this->tasks.empty()) return;
                    task = std::move(this->tasks.front());
                    this->tasks.pop();
                }
                task();
                progress++;
            }
        });
    }
}

extern atomic<int> failed;

void ThreadPool::startProgressDisplay(int lim) {
    limit = lim;
    if(lim==0)return;
    auto start_time = std::chrono::steady_clock::now();// 开始计时
    progress_thread = std::thread([this, lim, start_time] {
        while (true) {
            if (display_stop_requested) break;
            int current = progress.load();
            int per = (100 * current) / lim;
            string out = "\r[";
            bool boolOut = false;
            for (int i = 0; i < 101; i+=2) {
                int interval = per<10?1:(per<100?2:3);
                if (i < per) out += "■";
                else if(i == per || !boolOut){
                    out += to_string(per) + '%';
                    boolOut = true;
                }
                else if(i > per + interval)out += ' ';
            }
            out += "] (" + to_string(current) + "/" + to_string(lim) + ") ";

            // 计算每秒完成的任务数量
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();
            double tasks_per_second = elapsed > 0 ? current * 1000 / static_cast<double>(elapsed) : 0;
            out += to_string(tasks_per_second) + " tasks/sec [" + to_string(failed.load()) + " failed]";

            if (current >= lim) break;
            safeCOUTRaw(out);
            std::this_thread::sleep_for(std::chrono::milliseconds(2100));
        }
    });
}

void ThreadPool::stopProgressDisplay() {
//    safeCOUT("Progress display stopped");
    display_stop_requested = true;// 请求停止显示进度
    if (progress_thread.joinable()) {
        progress_thread.join();// 等待进度显示线程结束
    }
}

ThreadPool::~ThreadPool() {
    {
        std::unique_lock<std::mutex> lock(queue_mutex);
        stop = true;
    }
    condition.notify_all();
    for (std::thread &worker: workers) worker.join();
    stopProgressDisplay();
}