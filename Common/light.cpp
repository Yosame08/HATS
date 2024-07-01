//
// Created by yosame on 24-6-15.
//

#include "light.h"
#include "safeIO.h"
#include <fstream>
#include <sstream>
#include <cmath>
#include <cassert>
using namespace std;

static const float trSpeed = 3, slow_lim = 1;

static void revise(int &x) {
    if (x < 0) x += 86400;
    else if (x >= 86400)
        x -= 86400;
}

static void revise(long long &x) {
    if (x < 0) x += 86400;
    else if (x >= 86400)
        x -= 86400;
}

struct Last{
    int trajid,from,to;
    double stamp,elapsed,dist;
};

void Light::Traffic::modify(int l, int r, int val) {
    int correction = val;
    if(l<=r){
        if(static_cast<int>(score[l])+correction>127)correction = 127-score[l];
        else if(static_cast<int>(score[l])+correction<-127)correction = -127-score[l];
        if(static_cast<int>(score[r+1])-correction>127)correction = -(127-score[r+1]);
        else if(static_cast<int>(score[r+1])-correction<-127)correction = -(-127-score[r+1]);
        score[l]+=correction, score[r + 1]-=correction;
    }
    else{
        if(static_cast<int>(score[l])+correction>127)correction = 127-score[l];
        else if(static_cast<int>(score[l])+correction<-127)correction = -127-score[l];
        if(static_cast<int>(score[0])+correction>127)correction = 127-score[0];
        else if(static_cast<int>(score[0])+correction<-127)correction = -127-score[0];
        if(static_cast<int>(score[r+1])-correction>127)correction = -(127-score[r+1]);
        else if(static_cast<int>(score[r+1])-correction<-127)correction = -(-127-score[r+1]);
        score[l]+=correction, score[0]+=correction, score[r + 1]-=correction;
    }
    assert(abs(correction)<=2);
}

void Light::addInterval(int id, int to, double dl, double dr, int val, int mode) {
    int l = round(dl), r = round(dr);
    revise(l),revise(r);
    if(l>r&&!(l>=86300&&r<=100)){
        safeCERR("(Maybe bug)in addInterval, mode = "+to_string(mode) + " l="+to_string(l)+", r="+to_string(r)); // "l>r" is not allowed
        return;
    }
    if(!lights[id].count(to))lights[id][to] = new Traffic();
    auto &modify = lights[id][to];
    modify->modify(l, r, val);
    modify->percent+=val<0?val*2:val;
    modify->cnt+=abs(val);
}

void Light::parse(const string &traceFile) {
    ifstream dataFile(traceFile);
    string line;
    getline(dataFile, line); // skip the first line
    Last last{-1,-1,-1,-1,-1,-1};
    bool stopped=false, recovered=false;
    int lineCount = 0;
    safeCOUT("Parsing lights from " + traceFile);
    while(getline(dataFile, line)){
        ++lineCount;
        if(lineCount%1048576==0)safeCOUTRaw("\rParsed "+to_string(lineCount>>20)+"M lines");
        std::stringstream ss(line);
        string field;
        double in[6];
        bool ok=true;
        for(double & i : in){
            getline(ss, field, ',');
            i = stod(field);
            if(i<0){ // all parameters should be positive
                ok=false;
                break;
            }
        }
        if(!ok)continue;
        int id = static_cast<int>(in[trajID]), fromID = static_cast<int>(in[origin]), toID = static_cast<int>(in[transition]);
        double stamp = in[timestamps], passed = (int)in[elapsed];
        if(fromID!=toID && id==last.trajid){
            double bias = last.dist/trSpeed;
            if(fromID!=last.from){
                double vel = last.dist/last.elapsed; // "last" must reach the ending point of last road
                double mid = (last.stamp-bias+stamp+(stamp<last.stamp?86400:0))/2;
                if(stopped) addInterval(last.from, last.to, last.stamp-bias,stamp,2, 0);
                else if(vel < slow_lim){ // stop -> start in the same time interval
                    addInterval(last.from, last.to, last.stamp-bias, mid, -2, 1);
                    addInterval(last.from, last.to, mid,stamp,2, 2);
                }
                else addInterval(last.from, last.to, last.stamp+last.elapsed/2, stamp, 2, 3);
                stopped=recovered=false;
            }
            else{
                double vel = (last.dist-in[distance])/last.elapsed;
                if(vel >= -1 && in[distance]<=200){ // approaching crossing
                    int correction = in[distance]/trSpeed;
                    if(vel < slow_lim){ // when vehicle stops
                        stopped=true; recovered=false;
                        addInterval(fromID,toID,last.stamp-bias,stamp-correction,-1, 4);
                    }
                    else if((stopped && vel >= slow_lim) || recovered){ // when stopped vehicle moves
                        stopped=false; recovered=true;
                        addInterval(fromID,toID,last.stamp-bias,stamp-correction,1, 5);
                    }
                }
            }
        }
        else stopped=recovered=false;
        if(fromID>=lim)lim=fromID+1;
        last={id,fromID,toID,stamp,passed,in[distance]};
    }
    for(auto & light : lights)for(auto x:light)x.second->stat();
}

void Light::Traffic::stat() {
    int accu=0, transit=0;
    for(int i=0;i<86400;++i){
        accu+=score[i];
        if(accu>127) score[i] = static_cast<char>(accu&0x7f);
        else if(accu<-127) score[i] = static_cast<char>(accu%128);
        else score[i] = static_cast<char>(accu);
        if(accu>0)++transit;
        else --transit;
    }
    for(int i=0;i<86400;++i){
        int score_pst = 0, score_ftr = 0;
        for(int j=1;j<=3;++j){
            int past = i-j, future = i+j;
            revise(past), revise(future);
            score_pst += score[past], score_ftr += score[future];
        }
        if( abs(score_pst) > 6 && abs(score_ftr) > 6 && (score_pst ^ score_ftr)>>31 ){
            cross[i]=true;
        }
    }
    percent=1/(1+exp(-percent/cnt));//sigmoid
//    percent = (transit/86400.0/2)+0.5;
//    percent=1/(1+exp(-percent/cnt));//sigmoid
//    ofstream check("check.txt", ios::app);
//    if(cnt>=16384){
//        check<<cnt<<endl;
//        for(int i=0;i<=86400;++i)check<<score[i]<<' ';
//        check<<endl;
//    }
}

double Normal(double x){
    static double con = sqrt(M_PI*2);
    return 1/con*exp(-x*x/2);
}

double Light::query(int roadID, int toID, double timeDouble, double toNodeDist) const {
    long long timestamp = round(timeDouble - toNodeDist/trSpeed);
    revise(timestamp);
    if(toID==roadID || !lights[roadID].count(toID))return 1; //no data -> considering lights

    const auto &scoreArr = lights[roadID].at(toID)->score;
    const auto check = [&](int stamp){
        double result = 0;
        for(int bias=0;bias<=5;++bias){
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
    for(int i=timestamp-1, j=timestamp+1, cnt=0; cnt<300; ++cnt,--i,++j){
        revise(i);
        revise(j);
        if(lights[roadID].at(toID)->cross[i]&&pastCross==-1)pastCross=i;
        if(lights[roadID].at(toID)->cross[j]&&futureCross==-1)futureCross=j;
        if(pastCross!=-1&&futureCross!=-1)break;
    }
    if(pastCross!=-1&&futureCross!=-1){
        int cycle = (futureCross-pastCross+86400)%86400;
        int past=timestamp-cycle, future=timestamp+cycle;
        revise(past), revise(future);
        accu += check(past) * 0.25 + check(future) * 0.25;
    }
    double result = 1/(1+exp(-accu));
    // return result;
    return result == 0.5 ? lights[roadID].at(toID)->percent : result;
}
