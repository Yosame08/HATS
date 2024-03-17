#include "traffic.h"
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
    clog<<"Reading traffic data"<<endl;
    ifstream dataFile(filename);
    string inHead;
    getline(dataFile,inHead);
    string line;
    Last last{-1,-1,-1,-1,-1,-1};
    bool stopped=false;
    while(getline(dataFile, line)){
        static int cnt=0;
        if(++cnt%1048576==0)clog<<'\r'<<cnt/1048576<<"M Traffic lines";
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
        if(id==last.trajid){
            if(fromID!=last.from){
                stopped=false;
                addInterval(last.from,last.to,last.stamp+last.elapsed-2,last.stamp+last.elapsed+2,2);
//                double vel=last.dist/last.elapsed;
//                if(vel<1&&last.elapsed>5)
//                    addInterval(last.from,last.to,last.stamp-last.dist/5,last.stamp+last.elapsed/2-last.dist/5,-1);//stop->move
            }
            else{
                double vel = (last.dist-in[distance])/(stamp-last.stamp);
                if(vel<-2){
                    last={id,fromID,toID,stamp,passed,in[distance]};
                    continue; // anomaly
                }
                if(in[distance]<300){ // approaching crossing
                    if(vel<=1){ // when sig1 vehicle stops for 5+ sec
                        stopped=true;
                        int correction = in[distance]/5;
                        //if(last.stamp-correction>stamp-correction)cout<<"1 "<< last.stamp-correction << ' ' << stamp-correction<<endl;
                        addInterval(fromID,toID,last.stamp-correction,stamp-correction,-2);
                    }
                    else if(stopped && vel>2){ // when sig1 stopped vehicle moves
                        stopped=false;
                        //if(last.stamp-last.dist/5>stamp-in[distance]/5)cout<<"2 "<< last.stamp-last.dist/5 << ' ' << stamp-in[distance]/5;
                        addInterval(fromID,toID,last.stamp-last.dist/5,stamp-in[distance]/5,1);
                    }
                }
            }
        }else stopped=false;
        if(fromID>=lim)lim=fromID+1;
        last={id,fromID,toID,stamp,passed,in[distance]};
    }
    clog<<"\nPreprocessing..."<<endl;
    for(int i=0;i<=PATH_NUM;++i)for(auto x:lights[i])x.second->stat();
    clog<<"File read finish"<<endl;
}

double Normal(double x){
    static double con = sqrt(M_PI*2);
    return 1/con*exp(-x*x/2);
}

double TrafficHandler::query(int roadID, int toID, long long timestamp, float toNodeDist) const{
    timestamp -= toNodeDist/5;
    revise(timestamp);
    if(toID==roadID)return 0.5;//no considering lights
    if(!lights[roadID].count(toID))return 1/(1+exp(-1));
    double accu=0;
    const auto &scoreArr = lights[roadID].at(toID)->score;
    for(int bias=0;bias<=10;++bias){
        int l = timestamp - bias, r = timestamp + bias;
        revise(l), revise(r);
        double factor = Normal(bias);
        accu += scoreArr[l] * factor;
        if(l!=r)accu += scoreArr[r] * factor;
    }
    double result = 1/(1+exp(-accu));
    double certainty = abs(result-0.5), overall = lights[roadID].at(toID)->percent;
    return certainty > abs(overall-0.5) ? result : overall;
}

void TrafficHandler::Traffic::stat() {
    short accu=0;
    for(int i=0;i<=86400;++i){
        accu+=score[i];
        score[i]=accu;
    }
    percent=1/(1+exp(-percent/cnt));//sigmoid
}
