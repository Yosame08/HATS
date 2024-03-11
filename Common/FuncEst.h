#ifndef FUNCEST_H
#define FUNCEST_H

#include <string>
#include <vector>

double phi(double x);
double Estimate_wrap(double angle, int id);
void ReadTurn(const std::string &turnFN);
void FitParam();
void Output(const std::string &outFN);
std::vector<int> LoadParam(const std::string& paramFN);

#endif //FUNCEST_H
