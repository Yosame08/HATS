#ifndef FUNCEST_H
#define FUNCEST_H

#include <string>
#include <vector>

double phi(double x);
double Estimate_wrap(double val, int id, bool turn);
void ReadStat(const std::string &filename, bool turn);
void FitParam(bool turn);
void Output(const std::string &outFN, bool turn);
std::vector<int> LoadParam(const std::string& turnFN, const std::string& lenFN);

#endif //FUNCEST_H
