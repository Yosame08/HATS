//
// Created by yosame on 24-6-15.
//

#include "safeIO.h"
#include <iostream>
#include <mutex>
using namespace std;

mutex clog_mtx, cout_mtx, cerr_mtx;

void safeCOUTRaw(const string &message) {
    lock_guard<mutex> lock(cout_mtx);
    cout << message << flush;
}

void safeCOUT(const string &message) {
    lock_guard<mutex> lock(cout_mtx);
    cout << message << endl;
}

void safeCLOG(const string &message) {
    lock_guard<mutex> lock(clog_mtx);
    clog << '\r' << message << endl;
}

void safeCERR(const string &message) {
    lock_guard<mutex> lock(cerr_mtx);
    cerr << message << endl;
}
