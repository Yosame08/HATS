#ifndef READS_H
#define READS_H

#include "structs.h"
#include <unordered_map>
#include <string>

void ReadRoadNet(const std::string&, const std::string&, G&, Road[],
                 GridType&);
void ReadTraces(const std::string&, int&, std::vector<Trace>traces[], bool=false, bool=true);
void ReadTracesWithRoad(const std::string& traceFN, int&m, std::vector<TraceRD>traces[]);
int readIntsFromLine(std::ifstream& stream, std::vector<int>& numbers);
#endif //READS_H