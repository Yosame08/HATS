// Update at 2024.2.11
#include "opCSV.h"
#include<fstream>
#include<sstream>
#include<algorithm>
using namespace std;

CSVFile::CSVFile(const vector<std::string> &sHeaders) {
    int cnt=0;
    for(auto &i:sHeaders)headers[i]=cnt++;
    headers_seq=sHeaders;
}

CSVFile::CSVFile(const string& filename){
    ifstream file(filename);
    string line;
    getline(file, line);
    stringstream sHeaders(line);
    string header;
    int cnt=0;
    while(getline(sHeaders, header, ',')){
        header.erase(remove_if(header.begin(),header.end(), ::isspace),header.end());
        headers[header]=cnt++;
        headers_seq.push_back(header);
    }

    unsigned num = headers.size();
    while(getline(file,line)){
        data.emplace_back(this);
        data.back().item.reserve(num);
        stringstream s(line);
        string field;
        while(getline(s, field, ','))data.back().item.emplace_back(stod(field));
    }
}

unsigned CSVFile::size() const{
    return data.size();
}

unsigned CSVFile::headerSize()const{
    return headers.size();
}

double CSVFile::get(int id, const string &header){
    if(headers.find(header)==headers.end())throw string("Wrong header: ")+header;
    return data[id][header];
}

void CSVFile::set(int id, const string &header, double val){
    if(headers.find(header)==headers.end())throw string("Wrong header: ")+header;
    data[id][header]=val;
}

void CSVFile::saveTo(const string &path){
    ofstream out(path);
    for(auto it = headers_seq.begin(); it != headers_seq.end(); ++it){
        if(it != headers_seq.begin()) out << ",";
        out << *it;
    }
    out << "\n";
    for(auto &item : data){
        for(int i = 0; i < item.item.size(); ++i){
            if(i != 0) out << ",";
            out << (double)item.item[i];
        }
        out << "\n";
    }
}

void CSVFile::append(const vector<double> &line) {
    data.emplace_back(this);
    //data.back().item.reserve(headers.size());
    for(auto i:line)data.back().item.emplace_back(i);
}

double& CSVFile::Item::operator[](const string &header){
    return item[host->headers.at(header)];
}