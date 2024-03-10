#include "FuncEst.h"
#include "definitions.h"
#include <fstream>
#include <string>
#include <cmath>
#include <iostream>
#include <sstream>
#include <vector>
#include <set>
#include <thread>
#include <mutex>

using namespace std;

/*
 * This module is to estimate the probability of turning sig1 specified angle
 * We use 4 normal distribution to realize this function
 * 1/(sqrt(2*pi)*sig1)*exp(-x^2/(2*sig1^2))
 * when mu4 = 0 and 90 and 180
 */

vector<int> times;
const int mxSize = 90/granularity + 1;
const int mxMissing = 30;
vector<double> freq[mxMissing + 1];
enum pName{
    sig1,S1,sig2,mu2,S2,sig3,mu3,Size
};
double best_param[mxMissing + 1][Size], mulCache[mxMissing + 1][Size];
double min_loss[mxMissing + 1];
const double sqrt_2_PI = sqrt(M_PI*2);

void ReadTurn(const string &turnFN){
    std::ifstream file(turnFN);
    std::string line;
    int tot=0, timeNow;

    while (std::getline(file, line)) {
        std::istringstream iss(line);
        if (line.find("Secs:") != std::string::npos) {
            iss >> timeNow;
            times.emplace_back(timeNow);
            freq[tot].reserve(mxSize);
        } else {
            float num;
            int cnt=0;
            while (iss >> num) {
                ++freq[tot][int(num / timeNow / granularity)];
                ++cnt;
            }
            for(int i=0;i<mxSize;++i)freq[tot][i]/= cnt;
            ++tot;
        }
    }
    cout<<"Turning Info Read Finish"<<endl;
}

// code from: https://www.johndcook.com/blog/cpp_phi/
long double phi(long double x)
{
    // constants
    long double a1 =  0.254829592;
    long double a2 = -0.284496736;
    long double a3 =  1.421413741;
    long double a4 = -1.453152027;
    long double a5 =  1.061405429;
    long double p  =  0.3275911;

    // Save the sign of x
    int sign = 1;
    if (x < 0)
        sign = -1;
    x = fabs(x)/sqrtl(2.0);

    // A&S formula 7.1.26
    long double t = 1.0/(1.0 + p*x);
    long double y = 1.0 - (((((a5*t + a4)*t) + a3)*t + a2)*t + a1)*t*exp(-x*x);

    return 0.5*(1.0 + sign*y);
}

double CalcArea(long double _mu, long double _sigma){
    return phi((180-_mu)/_sigma) - phi(0-_mu/_sigma);
}

double Estimate(double x, const double param[], const double cache[]){
    const double x2 = x*x, x_mu_2 = (x-param[mu2]) * (x - param[mu2]), x_mu_3 = (x-param[mu3]) * (x-param[mu3]),
                 a2= param[sig1] * param[sig1], b2= param[sig2] * param[sig2], c2=param[sig3] * param[sig3];
    return cache[S1] * exp(-x2 / (a2 * 2))
         + cache[S2] * exp(-x_mu_2 / (b2 * 2))
         + cache[mu3] * exp(-x_mu_3 / (c2 * 2));
}

void FindOneParam(pName name, double areaLeft, int id, double param[], const double step[], double origin[]){
    double from = origin[name] - step[name] * 5, to = origin[name] + step[name] * 5;
    int cnt = 1;
    for(param[name]=from; param[name] <= to; param[name]+=step[name], ++cnt){
        if(param[name] <= 1e-6)continue;
        double use = 0, loss, sum=0;
        switch (name) {
            case mu3:
                mulCache[id][mu3] = areaLeft / CalcArea(param[mu3], param[sig3]) / (sqrt_2_PI * param[sig3]);
                loss = 0;
                for(int i=0;i<mxSize;++i){
                    double est = Estimate(i, param, mulCache[id]);
                    loss += abs(est-freq[id][i]);
                    sum+=est;
                }
                if(loss<min_loss[id]){
                    min_loss[id]=loss;
                    for(int i=0;i<Size;++i)best_param[id][i]=param[i];
                }
                continue;
            case S1:
                use = param[S1];
                mulCache[id][S1] = use / CalcArea(0, param[sig1]) / (sqrt_2_PI * param[sig1]);
                if(use>=areaLeft)return;
                break;
            case S2:
                use = param[S2];
                mulCache[id][S2] = use / CalcArea(param[mu2], param[sig2]) / (sqrt_2_PI * param[sig2]);
                if(use>=areaLeft)return;
                break;
            default:
                break;
        }
        FindOneParam(static_cast<pName>(name + 1), areaLeft-use, id, param, step, origin);
    }
}

mutex coutMutex;

void FindParam(int id){
    double origin_param[Size], step[Size], param[Size];
    static const double begin_param[] = {0.6, 0.6,2.5,  2.5,0.6, 20, 3};
    static const double begin_step[]  = {0.1,0.08,0.4,0.4,0.08,2.5,0.5};
    for(int i=0;i<Size;++i){
        origin_param[i]=begin_param[i];
        step[i]=begin_step[i];
        param[i]=origin_param[i];
    }
    for(int i=1;i<=8;++i){
        min_loss[id] = 1e300;
        FindOneParam(static_cast<pName>(0), 1, id, param, step, origin_param);
        for(int j=0;j<Size;++j){
            origin_param[j]=best_param[id][j];
            param[j]=origin_param[j];
            step[j]/=2;
        }
    }
    lock_guard<mutex> lock(coutMutex);
    cout << "Min loss:" << min_loss[id] << '\n';
}

void FitParam(){
    std::vector<std::thread> threads;
    threads.reserve(times.size());
    for(int i=0;i<times.size();++i){
        threads.emplace_back(FindParam, i);
    }
    for(auto &i:threads)i.join();
}

void Output(const string &outFN){
    ofstream out(outFN);
    for(int i=0;i<times.size();++i){
        out<<times[i]<<" Secs:\n";
        for(auto j:best_param[i])out<<j<<' ';
        out<<'\n';
    }
    out.close();
}

vector<int> LoadParam(const string& paramFN){
    std::ifstream file(paramFN);
    std::string line;
    int time,tot=0;

    while (std::getline(file, line)) {
        std::istringstream iss(line);
        if (line.find("Secs:") != std::string::npos) {
            iss >> time;
            times.push_back(time);
        } else {
            int cnt=0;
            while(iss >> best_param[tot][cnt++]);
            ++tot;
        }
    }

    for(int i=0;i<times.size();++i){
        mulCache[i][S1] = best_param[i][S1] / CalcArea(0,best_param[i][sig1]) / (sqrt_2_PI * best_param[i][sig1]);
        mulCache[i][S2] = best_param[i][S2] / CalcArea(best_param[i][mu2],best_param[i][sig2]) / (sqrt_2_PI * best_param[i][sig2]);
        double areaLeft = 1 - best_param[i][S1] - best_param[i][S2];
        mulCache[i][mu3] = areaLeft / CalcArea(best_param[i][mu3],  best_param[i][sig3]) / (sqrt_2_PI * best_param[i][sig3]);
    }
    clog<<"Params for predicting turning prob loaded"<<endl;
    //clog<<Estimate_wrap(0,8)<<' '<<Estimate_wrap(1,8)<<' '<<Estimate_wrap(10,8)<<' '<<Estimate_wrap(0,12)<<' '<<Estimate_wrap(1,12)<<' '<<Estimate_wrap(10,12)<<endl;
    set<int> ret;
    for(auto i:times)ret.insert(i);
    return times;
    //exit(0);
}

double Estimate_wrap(double angle, int id){
    return Estimate(angle/granularity, best_param[id], mulCache[id]);
}