#include "FuncEst.h"
#include "definitions.h"
#include <fstream>
#include <string>
#include <cmath>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <vector>
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
const int mxTurn = 4000 / granular_turn + 1, mxLen = 5000 / granular_len + 1;
const int mxMissing = 30;
vector<double> freqTurn[mxMissing + 1], freqLen[mxMissing + 1];
enum pName{
    sig1,S1,sig2,mu2,Size//,S2,sig3,mu3,Size
};
double bestTurn[mxMissing + 1][Size], cacheTurn[mxMissing + 1][Size], lossTurn[mxMissing + 1];
double bestLen[mxMissing + 1][Size], cacheLen[mxMissing + 1][Size], lossLen[mxMissing + 1];
const double sqrt_2_PI = sqrt(M_PI*2);

void ReadStat(const string &filename, bool turn){
    times.clear();
    auto &op = turn?freqTurn:freqLen;
    std::ifstream file(filename);
    std::string line;
    int tot=0, timeNow;
    float maxNum=0;

    while (std::getline(file, line)) {
        std::istringstream iss(line);
        if (line.find("Secs:") != std::string::npos) {
            iss >> timeNow;
            times.emplace_back(timeNow);
            op[tot].reserve(turn?mxTurn:mxLen);
        } else {
            float num;
            int cnt=0;
            while (iss >> num) {
                if(num>maxNum)maxNum=num;
                if(turn){
                    ++op[tot][int(num / granular_turn)];
                    ++cnt;
                }
                else if(num>=0){
                    ++op[tot][int(num / granular_len)];
                    ++cnt;
                }
            }
            for(int i=0; i < (turn?mxTurn:mxLen); ++i)op[tot][i]/= cnt;
            ++tot;
        }
    }
    cout<<"Information Read Finish, max = "<<maxNum<<endl;
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
    return 1 - phi(0-_mu/_sigma);
}

double Estimate(double x, const double param[], const double cache[]){
    const double x2 = x*x, x_mu_2 = (x-param[mu2]) * (x - param[mu2]),//, x_mu_3 = (x-param[mu3]) * (x-param[mu3]),
                 a2 = param[sig1] * param[sig1], b2 = param[sig2] * param[sig2];//, c2=param[sig3] * param[sig3];
    return cache[S1] * exp(-x2 / (a2 * 2))
         + cache[mu2] * exp(-x_mu_2 / (b2 * 2));
         //+ cache[mu3] * exp(-x_mu_3 / (c2 * 2));
}

double EstimateSP(double x, const double param[], const double cache[]){
    const double x2 = x*x, a2 = param[sig1] * param[sig1];
    return cache[S1] * exp(-x2 / (a2 * 2));
}

void FindOneParam(pName name, double areaLeft, int id, double param[], const double step[], double origin[], bool turn){
    auto &cacheArr = turn?cacheTurn[id]:cacheLen[id];
    auto &lossArr = turn?lossTurn:lossLen;
    auto &bestArr = turn?bestTurn:bestLen;
    const auto &freqArr = turn?freqTurn:freqLen;

    double from = origin[name] - step[name] * 11, to = origin[name] + step[name] * 11;
    int cnt = 1;
    for(param[name]=from; param[name] <= to; param[name]+=step[name], ++cnt){
        if(param[name] <= EPS)continue;
        double use = 0, loss, sum=0;
        switch (name) {
            case mu2:
                cacheArr[mu2] = areaLeft / CalcArea(param[mu2], param[sig2]) / (sqrt_2_PI * param[sig2]);
                loss = 0;
                for(int i=0; i < (turn?mxTurn:mxLen); ++i){
                    double est = Estimate(i, param, cacheArr);
                    loss += abs(est - freqArr[id][i]);
                    sum+=est;
                }
                if(loss < lossArr[id]){
                    lossArr[id]=loss;
                    for(int i=0;i<Size;++i)bestArr[id][i]=param[i];
                }
                continue;
            case S1:
                use = param[S1];
                cacheArr[S1] = use / CalcArea(0, param[sig1]) / (sqrt_2_PI * param[sig1]);
                if(use>=areaLeft)return;
                break;
//            case S2:
//                use = param[S2];
//                cacheTurn[id][S2] = use / CalcArea(param[mu2], param[sig2]) / (sqrt_2_PI * param[sig2]);
//                if(use>=areaLeft)return;
//                break;
            default:
                break;
        }
        FindOneParam(static_cast<pName>(name + 1), areaLeft-use, id, param, step, origin, turn);
    }
}

mutex coutMutex;

void FindParam(int id, bool turn){
    auto &bestArr = turn?bestTurn:bestLen;
    double origin_param[Size], step[Size], param[Size];
    static const double begin_param[] = {24,0.5, 240,120};//,0.6, 20, 3};
    static const double begin_step[]  = {2, 0.04,20, 10};//,0.08,2.5,0.5};
    for(int i=0;i<Size;++i){
        origin_param[i]=begin_param[i];
        step[i]=begin_step[i];
        param[i]=origin_param[i];
    }
    for(int i=1;i<=25;++i){
        if(turn)lossTurn[id] = 1e300;
        else lossLen[id] = 1e300;
        FindOneParam(static_cast<pName>(0), 1, id, param, step, origin_param, turn);
        for(int j=0;j<Size;++j){
            origin_param[j]=bestArr[id][j];
            param[j]=origin_param[j];
            step[j]/=1.5;
        }
    }
    lock_guard<mutex> lock(coutMutex);
    cout << id << ": Min loss = " << (turn?lossTurn[id]:lossLen[id]) << '\n';
}

void FitParam(bool turn){
    std::vector<std::thread> threads;
    threads.reserve(times.size());
    for(int i=0;i<times.size();++i){
        threads.emplace_back(FindParam, i, turn);
    }
    for(auto &i:threads)i.join();
}

void Output(const string &outFN, bool turn){
    ofstream out(outFN);
    out<<fixed<<setprecision(8);
    for(int i=0;i<times.size();++i){
        out<<times[i]<<" Secs:\n";
        for(auto j:(turn?bestTurn[i]:bestLen[i]))out << j << ' ';
        out<<'\n';
    }
    out.close();
}

vector<int> LoadParam(const string& turnFN, const string& lenFN){
    std::ifstream fileTurn(turnFN);
    std::string line;
    int time,tot=0;
    while (std::getline(fileTurn, line)) {
        std::istringstream iss(line);
        if (line.find("Secs:") != std::string::npos) {
            iss >> time;
            times.push_back(time);
        } else {
            int cnt=0;
            while(iss >> bestTurn[tot][cnt++]);
            ++tot;
        }
    }
    fileTurn.close();
    std::ifstream fileLen(lenFN);
    tot=0;
    while (std::getline(fileLen, line)) {
        std::istringstream iss(line);
        if (line.find("Secs:") != std::string::npos)iss >> time;
        else {
            int cnt=0;
            while(iss >> bestLen[tot][cnt++]);
            ++tot;
        }
    }

    for(int i=0;i<times.size();++i){
        cacheTurn[i][S1] = bestTurn[i][S1] / CalcArea(0, bestTurn[i][sig1]) / (sqrt_2_PI * bestTurn[i][sig1]);
        double areaLeft = 1 - bestTurn[i][S1];
        cacheTurn[i][mu2] = areaLeft / CalcArea(bestTurn[i][mu2], bestTurn[i][sig2]) / (sqrt_2_PI * bestTurn[i][sig2]);
        cacheLen[i][S1] = bestLen[i][S1] / CalcArea(0, bestLen[i][sig1]) / (sqrt_2_PI * bestLen[i][sig1]);
        areaLeft = 1 - bestLen[i][S1];
        cacheLen[i][mu2] = areaLeft / CalcArea(bestLen[i][mu2], bestLen[i][sig2]) / (sqrt_2_PI * bestLen[i][sig2]);
//        cacheTurn[i][S2] = bestTurn[i][S2] / CalcArea(bestTurn[i][mu2], bestTurn[i][sig2]) / (sqrt_2_PI * bestTurn[i][sig2]);
//        double areaLeft = 1 - bestTurn[i][S1] - bestTurn[i][S2];
//        cacheTurn[i][mu3] = areaLeft / CalcArea(bestTurn[i][mu3], bestTurn[i][sig3]) / (sqrt_2_PI * bestTurn[i][sig3]);
    }
    clog<<"Params for predicting turning prob loaded. All estimated values below should be in range (0,1):"<<endl;
    clog<<Estimate_wrap(0,16,true)<<' '<<Estimate_wrap(200,16,true)<<' '<<Estimate_wrap(600,16,true)
        <<' '<<Estimate_wrap(0,16,false)<<' '<<Estimate_wrap(200,16,false)<<' '<<Estimate_wrap(600,16,false)<<endl;
    return times;
    //exit(0);
}

double Estimate_wrap(double val, int id, bool turn){
    if(turn) return Estimate(val / granular_turn, bestTurn[id], cacheTurn[id]);
    else if(val>=0)return Estimate(val / granular_len, bestLen[id], cacheLen[id]);
    else return EstimateSP(val / granular_len, bestLen[id], cacheLen[id]);
}