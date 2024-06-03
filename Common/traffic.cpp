#include "traffic.h"
#include "funcIO.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <cmath>
using namespace std;
static const float trSpeed = 3, slow_lim = 1;

void revise(int &x) {
    if(x<0)x+=86400;
    else if(x>86400)x-=86400;
}

void revise(long long &x) {
    if(x<0)x+=86400;
    else if(x>86400)x-=86400;
}

void TrafficHandler::addInterval(int id, int to, int l, int r, short val, int mode) {
    revise(l),revise(r);
    if(l>r&&!(l>=86300&&r<=100)){
        std::cerr<<"l="<<l<<", r="<<r<<endl; // "l>r" is not allowed
        std::cerr<<"(Maybe bug)in addInterval, mode = "<<mode<<endl;
        return;
    }
    if(!lights[id].count(to))lights[id][to] = new Traffic();
    auto &modify = lights[id][to];
    if(l<=r)modify->score[l]+=val, modify->score[r + 1]-=val;
    else modify->score[l]+=val, modify->score[0]+=val, modify->score[r + 1]-=val;
    modify->percent+=val<0?val*2:val;
    modify->cnt+=abs(val);
}

int statcnt = 0;
void TrafficHandler::init(const char* filename) {
    safe_cout("Reading traffic data");
    ifstream dataFile(filename);
    string inHead;
    getline(dataFile,inHead);
    string line;
    Last last{-1,-1,-1,-1,-1,-1};
    bool stopped=false, recovered=false;
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
            int bias = last.dist/trSpeed;
            if(fromID!=last.from){
                double vel=last.dist/last.elapsed; // must be at the beginning of new road
                int mid = last.stamp+(last.elapsed-bias)/2;
                if(stopped) addInterval(last.from, last.to, last.stamp-bias,stamp,2, 0);
                else if(vel < slow_lim && last.elapsed > 1){ // stop -> start in the same time interval
                    addInterval(last.from, last.to, last.stamp-bias, mid-1, -2, 1);
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
    safe_cout("\nPreprocessing...");
    for(int i=0;i<=PATH_NUM;++i)for(auto x:lights[i])x.second->stat();
    safe_cout("File read finish");
    safe_cout(to_string(statcnt)+" cross points");
}

double Normal(double x){
    static double con = sqrt(M_PI*2);
    return 1/con*exp(-x*x/2);
}

double TrafficHandler::query(int roadID, int toID, long long timestamp, float toNodeDist) const{
    timestamp -= toNodeDist/trSpeed;
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

void TrafficHandler::Traffic::stat() {
    short accu=0;
    for(int i=0;i<=86400;++i){
        accu+=score[i];
        score[i]=accu;
    }
    for(int i=0;i<=86400;++i){
        int score_pst = 0, score_ftr = 0;
        for(int j=1;j<=3;++j){
            int past = i-j, future = i+j;
            revise(past), revise(future);
            score_pst += score[past], score_ftr += score[future];
        }
        if( abs(score_pst) > 6 && abs(score_ftr) > 6 && (score_pst ^ score_ftr)>>31 ){
            cross[i]=true;
            ++statcnt;
        }
    }
    percent=1/(1+exp(-percent/cnt));//sigmoid

}
