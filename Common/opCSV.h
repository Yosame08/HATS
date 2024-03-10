// Update at 2024.2.11
#include<string>
#include<vector>
#include<unordered_map>
class CSVFile{
private:
    struct Item{
        CSVFile *host;
        std::vector<double>item;
        explicit Item(CSVFile *h): host(h){}
        double& operator[](const std::string &header);
        template<typename... Args>
        void push_back(double first, Args... args){
            item.emplace_back(first);
            if constexpr (sizeof...(args) > 0)push_back(args...);
        }
        template<typename T, typename... Args>
        void push_back(const std::vector<T> &first, Args... args){
            for(auto x:first)item.emplace_back(x);
            if constexpr (sizeof...(args) > 0)push_back(args...);
        }
    };
public:
    explicit CSVFile(const std::vector<std::string>& headers);
    explicit CSVFile(const std::string& filename);
    unsigned size()const;
    unsigned headerSize()const;
    double get(int id, const std::string &header);
    void set(int id, const std::string &header, double val);
    void saveTo(const std::string& path);
    void append(const std::vector<double> &line);
    Item& back() { return data.back(); }
    Item& operator[](int i) { return data[i]; }
    std::vector<Item>::iterator begin() { return data.begin(); }
    std::vector<Item>::iterator end() { return data.end(); }
private:
    std::vector<std::string>headers_seq;
    std::unordered_map<std::string,int>headers;
    std::vector<Item>data;
};
