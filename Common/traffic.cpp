#include "traffic.h"
#include "funcIO.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <cmath>
using namespace std;

void revise(int &x) {
    if(x<0)x+=86400;
    else if(x>86400)x-=86400;
}

void revise(long long &x) {
    if(x<0)x+=86400;
    else if(x>86400)x-=86400;
}

void TrafficHandler::addInterval(int id, int to, int l, int r, short val) {
    revise(l),revise(r);
    if(l>r&&!(l>86200&&r<200)){
        std::cerr<<"(Bug)Error in addInterval"<<endl;
        return;
    }
    if(!lights[id].count(to))lights[id][to] = new Traffic();
    auto &modify = lights[id][to];
    if(l<=r)modify->score[l]+=val, modify->score[r + 1]-=val;
    else modify->score[l]+=val, modify->score[0]+=val, modify->score[r + 1]-=val;
    modify->percent+=val;
    modify->cnt+=abs(val);
}

void TrafficHandler::init(const char* filename) {
    safe_cout("Reading traffic data");
    ifstream dataFile(filename);
    string inHead;
    getline(dataFile,inHead);
    string line;
    Last last{-1,-1,-1,-1,-1,-1};
    bool stopped=false;
    while(getline(dataFile, line)){
        static int cnt=0;
        if(++cnt%1048576==0)safe_cout_origin("\r"+to_string(cnt/1048576)+"M Traffic lines");
        std::stringstream ss(line);
        string field;
        double in[7];
        bool ok=true;
        for(int i=0;i<=6;++i){
            getline(ss, field, ',');
            in[i] = stod(field);
            if(in[i]<0){ // all parameters should be positive
                ok=false;
                break;
            }
        }
        if(!ok)continue;
        int id = (int)in[trajid], fromID = (int)in[original], toID = (int)in[transition], h = (int)in[hour], s = (int)in[sec];
        int stamp = h*3600+s, passed = (int)in[elapsed];
        if(fromID!=toID && id==last.trajid){
            int bias = last.dist/5*2;
            if(fromID!=last.from){
                stopped=false;
                double vel=last.dist/last.elapsed; // must be at the beginning of new road
                if(vel < 2){ // stop -> start
                    addInterval(last.from,last.to,last.stamp - bias,last.stamp - bias + last.elapsed/2,-1);
                }
                addInterval(last.from,last.to,last.stamp - bias + last.elapsed/2,stamp,1);
            }
            else{
                double vel = (last.dist-in[distance])/last.elapsed;
                if(vel < -2){
                    last={id,fromID,toID,stamp,passed,in[distance]};
                    continue; // anomaly
                }
                if(in[distance]<200){ // approaching crossing
                    int correction = in[distance]/5*2;
                    if(vel <= 2){ // when vehicle stops
                        stopped=true;
                        addInterval(fromID,toID,last.stamp-bias,stamp-correction,-1);
                    }
                    else if(stopped && vel>2){ // when stopped vehicle moves
                        stopped=false;
                        addInterval(fromID,toID,last.stamp-bias,stamp-correction,1);
                    }
                }
            }
        }
        else stopped=false;
        if(fromID>=lim)lim=fromID+1;
        last={id,fromID,toID,stamp,passed,in[distance]};
    }
    safe_cout("\nPreprocessing...");
    for(int i=0;i<=PATH_NUM;++i)for(auto x:lights[i])x.second->stat();
    safe_cout("File read finish");
}

double Normal(double x){
    static double con = sqrt(M_PI*2);
    return 1/con*exp(-x*x/2);
}

double TrafficHandler::query(int roadID, int toID, long long timestamp, float toNodeDist) const{
    // timestamp -= toNodeDist/5*1.5;
    revise(timestamp);
    if(toID==roadID || !lights[roadID].count(toID))return 1; //no considering lights

    const auto &scoreArr = lights[roadID].at(toID)->score;
    const auto check = [&](int stamp){
        double result = 0;
        for(int bias=0;bias<=10;++bias){
            int l = stamp - bias, r = stamp + bias;
            revise(l), revise(r);
            double factor = Normal(bias);
            result += scoreArr[l] * factor;
            if(l!=r)result += scoreArr[r] * factor;
        }
        return result;
    };
    double accu=check(timestamp);
    int pastCross=-1, futureCross=-1;
    for(int i=timestamp-1,cnt=0;cnt<300;++cnt,--i){
        revise(i);
        if(lights[roadID].at(toID)->cross[i]){
            pastCross=i;
            break;
        }
    }
    for(int i=timestamp+1,cnt=0;cnt<300;++cnt,++i){
        revise(i);
        if(lights[roadID].at(toID)->cross[i]){
            futureCross=i;
            break;
        }
    }
    if(pastCross!=-1&&futureCross!=-1){
        int cycle = (futureCross-pastCross+86400)%86400;
        int past=timestamp-cycle, future=timestamp+cycle;
        revise(past), revise(future);
        accu += check(past) * 0.25 + check(future) * 0.25;
    }
    double result = 1/(1+exp(-accu));
    return result == 0.5 ? lights[roadID].at(toID)->percent : result;
}

void TrafficHandler::Traffic::stat() {
    short accu=0;
    for(int i=0;i<=86400;++i){
        accu+=score[i];
        score[i]=accu;
    }
    for(int i=0;i<=86400;++i){
        int past = i-1, future = i+1;
        revise(past), revise(future);
        if( score[past] && score[future] && ((score[past]^score[future])>>31) ){
            cross[i]=true;
        }
    }
    percent=1/(1+exp(-percent/cnt));//sigmoid
}
