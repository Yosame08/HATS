#include "FuncEst.h"
#include "structs.h"
#include "funcIO.h"
#include <fstream>
#include <string>
#include <cmath>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <cassert>
using namespace std;

/*
 * This module is to estimate the probability of turning and length
 * We use 2 normal distribution to implement this function
 */

const double sqrt_2_PI = sqrt(M_PI*2);
const int width = 10; //should be int, ">>" used
void FunctionFit::ReadStat(const string &filename, bool rev){
    safe_cout("Reading statistics from "+filename);
    std::ifstream file(filename);
    std::string line;
    int tot=0, timeNow;
    float maxNum=0;

    while (std::getline(file, line)) {
        std::istringstream iss(line);
        if (line.find("Secs:") != std::string::npos) {
            iss >> timeNow;
            times.emplace_back(timeNow);
        }
        else {
            vector<float> ins;
            float num;
            while (iss >> num) {
                if(rev)num=-num;
                num/=granularity;
                if(num<0||(rev&&num==0))continue;
                ins.push_back(num);
                if(num>maxNum)maxNum=num;
            }
            sort(ins.begin(),ins.end());
            long accept = ins.size() * 0.999, begins = std::upper_bound(ins.begin(), ins.end(), 0) - ins.begin();
            auto ratio2index = [&](double ratio){
                long len = accept - begins;
                return begins + int(len * ratio);
            };
            sigMul1[tot] = ins[ratio2index(0.6827*0.5)], sigMul3[tot] = ins[ratio2index((0.9973+1)/2)], midVal[tot] = ins[ratio2index(0.75)];
            safe_cout("[Fit] Interval = "+to_string(times.back())+"s, Max: "+to_string(ins.back())+", 99.9%: "+to_string(ins[accept-1]));
            upLim[tot] = ins[accept-1]+1;
            stat[tot].reserve(upLim[tot]+1);
            for(int i=0;i<=upLim[tot];++i)stat[tot][i]=0;
            for(int i=0;i<accept;++i){
                if(int(ins[i])>upLim[tot])break;
                ++stat[tot][int(ins[i])];
            }
            for(int i=0;i<=upLim[tot]; ++i){
                stat[tot][i]/=accept;
                assert(stat[tot][i]<1);
            }
            ++tot;
        }
    }
}

// code from: https://www.johndcook.com/blog/cpp_phi/
double phi(double x)
{
    // constants
    double a1 =  0.254829592;
    double a2 = -0.284496736;
    double a3 =  1.421413741;
    double a4 = -1.453152027;
    double a5 =  1.061405429;
    double p  =  0.3275911;

    // Save the sign of x
    int sign = 1;
    if (x < 0)
        sign = -1;
    x = fabs(x)/sqrtl(2.0);

    // A&S formula 7.1.26
    double t = 1.0/(1.0 + p*x);
    double y = 1.0 - (((((a5*t + a4)*t) + a3)*t + a2)*t + a1)*t*exp(-x*x);

    return 0.5*(1.0 + sign*y);
}

double CalcArea(double _mu, double _sigma){
    return 1 - phi(0-_mu/_sigma);
}

double FunctionFit::Estimate(double x, const double param[], const double cache[]) {
    const double x_mu_2 = (x-param[mu2]) * (x-param[mu2]);
    return cache[S1] * exp(-x*x / cache[sig1]) + cache[mu2] * exp(-x_mu_2 / cache[sig2]);
}

void FunctionFit::EstiUpdate(int id, const double param[]){
    double l=0, sum=0;
    for(int i=0; i<=upLim[id]; ++i){
        double est = Estimate(i, param, cache[id]);
        if(i==0) est += param[val0];
        sum += est;
        prep[id][i] = est;
    }
    for(int i=0; i<=upLim[id]; ++i){
        double est = prep[id][i]/sum;
        l += abs(stat[id][i]-est);
        if(l >= loss[id])break;
    }
    if(l < loss[id]){
        loss[id] = l;
        for(int i=0;i<Size;++i)params[id][i] = param[i];
    }
}

void FunctionFit::FindPreciseParam(int id, const double step[], const double ori[]){
    auto &cacheArr = cache[id];
    vector<double>paramList[Size];
    for(int i=-width;i<=width;++i){
        for(int ty=0;ty<Size;++ty){
            double now = ori[ty]+step[ty]*i;
            if(now<=1e-10)continue;
            paramList[ty].push_back(now);
        }
    }

    double param[Size];
    for (auto v0: paramList[val0]){
        if(v0>=1)continue;
        param[val0] = v0;
        for (auto v1: paramList[sig1]) {
            param[sig1] = v1;
            cacheArr[sig1] = param[sig1] * param[sig1] * 2;
            for (auto v2: paramList[S1]) {
                if(v2+v0>=1)continue;
                param[S1] = v2;
                cacheArr[S1] = param[S1] / CalcArea(0, param[sig1]) / (sqrt_2_PI * param[sig1]);
                for (auto v3: paramList[sig2]) {
                    param[sig2] = v3;
                    cacheArr[sig2] = param[sig2] * param[sig2] * 2;
                    for (auto v4: paramList[mu2]) {
                        param[mu2] = v4;
                        cacheArr[mu2] = (1 - param[val0] - param[S1]) / CalcArea(param[mu2], param[sig2]) / (sqrt_2_PI * param[sig2]);
                        EstiUpdate(id, param);
                    }
                }
            }
        }
    }

}

void FunctionFit::FindParam(int id){
    double origin_param[Size], step[Size];
    params[id][val0] = 0.1;
    params[id][sig1] = sigMul1[id];
    params[id][S1] = 0.45;
    params[id][sig2] = (sigMul3[id]-midVal[id]) / 3;
    params[id][mu2] = midVal[id];
    safe_cout(to_string(id)+" initial params: "+to_string(params[id][0])+" "+to_string(params[id][1])
    +" "+to_string(params[id][2])+" "+to_string(params[id][3])+ " " + to_string(params[id][4]));
    for(int i=0;i<Size;++i)step[i] = params[id][i]/width;

    auto updateParam = [&](){
        for(int i=0;i<Size;++i)origin_param[i] = params[id][i];
    };
    updateParam();

    for(int i=1;i<=15;++i){
        if(i==8)safe_cout(to_string(id)+" Finding parameters... 50%");
        FindPreciseParam(id, step, origin_param);
        for(int j=0;j<Size;++j){
            if(params[id][j] <= origin_param[j]-step[j]*(width>>1) || params[id][j] >= origin_param[j]+step[j]*(width>>1))continue;
            step[j] /= 2;
        }
        updateParam();
    }
    stringstream str;
    str << id << ": Min loss = " << loss[id];
    safe_cout(str.str());
}

void FunctionFit::FitParam(int threads){
    ThreadPool pool(threads);
    for(int i=0;i<times.size();++i){
        loss[i] = 1e308;
        pool.enqueue([i, this](){ FindParam(i); }); // 将任务添加到线程池
    }
}

void FunctionFit::Output(const string &filename){
    ofstream out(filename);
    out<<fixed<<setprecision(10);
    for(int i=0;i<times.size();++i){
        out<<times[i]<<" Secs "<<upLim[i]<<"\n";
        for(auto j:params[i])out << j << ' ';
        out<<'\n';
    }
    out.close();
}

void FunctionFit::LoadParam(const std::string& filename){
    std::ifstream file(filename);
    std::string line;
    int time,tot=0;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        if (line.find("Secs") != std::string::npos) {
            string tmp;
            int lim;
            iss >> time >> tmp >> lim;
            times.push_back(time);
            upLim[tot] = lim;
        } else {
            int cnt=0;
            while(iss >> params[tot][cnt++]);
            ++tot;
        }
    }
    file.close();
    for(int i=0;i<times.size();++i){
        cache[i][S1] = params[i][S1] / CalcArea(0, params[i][sig1]) / (sqrt_2_PI * params[i][sig1]);
        cache[i][mu2] = (1 - params[i][S1]) / CalcArea(params[i][mu2], params[i][sig2]) / (sqrt_2_PI * params[i][sig2]);
        cache[i][sig1] = params[i][sig1] * params[i][sig1] * 2;
        cache[i][sig2] = params[i][sig2] * params[i][sig2] * 2;
    }
    cout<<"upper limit = ";
    for(int i=0;i<times.size();++i){
        for(int x=0;;++x)if(Estimate(x,params[i],cache[i])<1e-40){
            upLim[i]=x;
            cout<<x<<' ';
            break;
        }
    }
    cout<<endl;
    for(int i=0;i<times.size();++i){
        double sum = 0;
        for(int x=upLim[i];x>=0;x-=1){
            sum+=Estimate(x, params[i], cache[i]);
            prep[i][x]=sum;
            if(x==0)prep[i][x]+=params[i][val0], sum+=params[i][val0];
        }
        for(int x=0;x<=upLim[i];x+=1){
            prep[i][x]/=sum;
            if(prep[i][x]<minProb)prep[i][x]=minProb;
        }
    }

    safe_cout("Parameters loaded. All estimated values below should be in range [0,1] (, or it is a bug):");
    safe_cout(to_string(Estimate_wrap(0,8))+" "+to_string(Estimate_wrap(10,12))+" "+to_string(Estimate_wrap(100,16)));
}

double FunctionFit::Estimate_wrap(double val, int id) const{
    int v = int(abs(val/granularity));
    if(v>=upLim[id])return prep[id][upLim[id]];
    return prep[id][v];
    //return Estimate(val, params[id], cache[id]);
}
