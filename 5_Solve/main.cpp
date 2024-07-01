#include "solve.h"
Solve solve(Final);

int main(int argc, char *argv[]) {
    int num_threads = 4;
    std::string recoverName = "recovery";// default value
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-th") == 0 && i + 1 < argc) {
            num_threads = std::stoi(argv[i + 1]);
            if (num_threads < 1) num_threads = 1;
            ++i;// skip next argument
        } else if (strcmp(argv[i], "-fn") == 0 && i + 1 < argc) {
            recoverName = argv[i + 1];
            ++i;// skip next argument
        } else
            std::cout << "redundant argument: " << argv[i] << std::endl;
    }
    solve.initRoadNet("../../Map/edgeOSM.txt", "../../Map/wayTypeOSM.txt");
    solve.readTraces("../../Dataset/test_input.txt", true, false);
    solve.initTraffic("../../Intermediate/params");
    solve.initLight("../../Intermediate/train_preprocess.csv");
    solve.initModel("../../Intermediate/road_vectors.txt", "../../Intermediate/model_vel.pt");
    solve.multiSolve(num_threads);
    solve.outputRecovery(recoverName);
    solve.outputMatching(recoverName);
    return 0;
}
