#include "solve.h"
Solve solve(Preprocess);

int main(int argc, char *argv[]) {
    int num_threads = 4;
    std::string mode = "train"; // default value
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-th") == 0 && i + 1 < argc) {
            num_threads = std::stoi(argv[i + 1]);
            if(num_threads<1)num_threads=1;
            ++i; // skip next argument
        } else if (strcmp(argv[i], "-v") == 0) {
            mode = "valid";
        } else if (strcmp(argv[i], "-t") == 0) {
            mode = "train";
        }
        else std::cout << "redundant argument: " << argv[i] << std::endl;
    }

    solve.initRoadNet(MapPath+std::string("edgeOSM.txt"), MapPath+std::string("wayTypeOSM.txt"));
    solve.readTraces(DatasetPath+mode+"_input.txt", true, true);
    solve.emptyTraffic();
    solve.multiSolve(num_threads);
    solve.outputPreprocess(mode);
    solve.outputMatching(mode);
    return 0;
}
