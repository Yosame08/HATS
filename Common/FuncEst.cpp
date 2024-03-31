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
void FunctionFit::ReadStat(const string &filename, bool rev){
    std::ifstream file(filename);
    std::string line;
    int tot=0, timeNow;
    float maxNum=0;

    while (std::getline(file, line)) {
        std::istringstream iss(line);
        if (line.find("Secs:") != std::string::npos) {
            iss >> timeNow;
            times.emplace_back(timeNow);
            stat[tot].reserve(upLim+1);
            for(int i=0;i<=upLim;++i)stat[tot][i]=0;
        }
        else {
            vector<float> ins;
            float num;
            while (iss >> num) {
                if(rev)num=-num;
                if(num<0)continue;
                ins.push_back(num);
                if(num>maxNum)maxNum=num;
            }
            sort(ins.begin(),ins.end());
            int accept = ins.size() * 0.999;
            clog<<"[FunctionFit] Time = "<<times.back()<<", Origin max: "<<ins.back()<<", 99.9% max: "<<ins[accept-1]<<endl;
            for(int i=0;i<accept;++i){
                if(int(ins[i])>upLim)break;
                ++stat[tot][int(ins[i])];
            }
            for(int i=0;i<=upLim; ++i){
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
    const double x2 = x*x, x_mu_2 = (x-param[mu2]) * (x - param[mu2]);//, x_mu_3 = (x-param[mu3]) * (x-param[mu3]),
    return cache[S1] * exp(-x2 / cache[sig1])
         + cache[mu2] * exp(-x_mu_2 / cache[sig2]);
}

void FunctionFit::EstiUpdate(int id, const double param[]){
    double l=0;
    for(int i=0; i<=upLim; ++i){
        double est = Estimate(i, param, cache[id]);
        assert(est==est); // nan inf
        l += abs(est - stat[id][i]);
    }
    if(l < loss[id]){
        loss[id] = l;
        for(int i=0;i<Size;++i)params[id][i] = param[i];
    }
}

const int width = 10;
void FunctionFit::FindPreciseParam(int id, double param[], const double step[], const double ori[]){
    auto &cacheArr = cache[id];
    for(double v1=ori[0]-step[0]*width; v1<=ori[0]+step[0]*width; v1+=step[0]) {
        if(v1<=1e-10)continue;
        param[sig1] = v1;
        cacheArr[sig1] = param[sig1] * param[sig1] * 2;
        for(double v2=ori[1]-step[1]*width; v2<=ori[1]+step[1]*width; v2+=step[1]) {
            if(v2<=1e-10)continue;
            param[S1] = v2;
            cacheArr[S1] = param[S1] / CalcArea(0, param[sig1]) / (sqrt_2_PI * param[sig1]);
            for (double v3=ori[2]-step[2]*width; v3<=ori[2]+step[2]*width; v3+=step[2]) {
                if(v3<=1e-10)continue;
                param[sig2] = v3;
                cacheArr[sig2] = param[sig2] * param[sig2] * 2;
                for (double v4=ori[3]-step[3]*width; v4<=ori[3]+step[3]*width; v4+=step[3]) {
                    if(v4<=1e-10)continue;
                    param[mu2] = v4;
                    cacheArr[mu2] = (1 - param[S1]) / CalcArea(param[mu2], param[sig2]) / (sqrt_2_PI * param[sig2]);
                    EstiUpdate(id, param);
                }
            }
        }
    }
}

void FunctionFit::FindParam(int id){
    double origin_param[Size] = {151, 0.5, 151, 151};
    double tmp_param[Size]    = {151, 0.5, 151, 151};
    auto updateParam = [&](){
        for(int i=0;i<Size;++i)tmp_param[i] = origin_param[i] = params[id][i];
    };
    updateParam();
    double step[] = {15, 0.05,15, 15};
    safe_cout(to_string(id)+" Finding parameters... 0%");
    for(int i=1;i<=20;++i){
        //auto bg = clock();
        if(i==11)safe_cout(to_string(id)+" Finding parameters... 50%");
        FindPreciseParam(id, tmp_param, step, origin_param);
        for(int j=0;j<Size;++j){
            if(params[id][j] <= origin_param[j]-step[j]*(width-5) || params[id][j] >= origin_param[j]+step[j]*(width-5))continue;
            step[j] /= 4;
        }
        updateParam();
        //safe_clog("Epoch time" + to_string((clock()-bg)/(double)CLOCKS_PER_SEC));
    }
    stringstream str;
    str << id << ": Min loss = " << loss[id];
    safe_cout(str.str());
}

void FunctionFit::FitParam(){
    ThreadPool pool(15);
    for(int i=0;i<times.size();++i){
        loss[i] = 1e308;
        pool.enqueue([i, this](){ FindParam(i); }); // 将任务添加到线程池
    }
}

void FunctionFit::Output(const string &filename){
    ofstream out(filename);
    out<<fixed<<setprecision(10);
    for(int i=0;i<times.size();++i){
        out<<times[i]<<" Secs:\n";
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
        if (line.find("Secs:") != std::string::npos) {
            iss >> time;
            times.push_back(time);
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

    for(int i=0;i<times.size();++i){
        double sum = 0;
        for(int x=upLim;x>=0;x-=1){
            sum+=Estimate(x, params[i], cache[i]);
            prep[i][x]=sum;
        }
        for(int x=0;x<=upLim;x+=1)prep[i][x]/=sum;
    }

    clog<<"Parameters loaded. All estimated values below should be in range (0,1) (, or it is a bug):"<<endl;
    clog<<Estimate_wrap(0,8)<<' '<<Estimate_wrap(300,12)<<' '<<Estimate_wrap(2000,16)<<endl;
}

double FunctionFit::Estimate_wrap(double val, int id) const{
    return prep[id][int(abs(val))];
    //return Estimate(val, params[id], cache[id]);
}