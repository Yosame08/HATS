#include "funcIO.h"
#include <mutex>
#include <iostream>
using namespace std;
mutex clog_mtx, cout_mtx, cerr_mtx;
void safe_clog(const string& message){
    lock_guard<mutex> lock(clog_mtx);
    clog << '\r' << message << endl;
}
void safe_cout(const string& message){
    lock_guard<mutex> lock(cout_mtx);
    cout << message << endl;
}
void safe_cerr(const string& message){
    lock_guard<mutex> lock(cerr_mtx);
    cerr << message << endl;
}
void safe_cout_origin(const string& message){
    lock_guard<mutex> lock(cout_mtx);
    cout << message;
}