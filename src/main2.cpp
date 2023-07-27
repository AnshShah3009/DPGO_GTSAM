#include <iostream>
#include "gtsam/geometry/Pose3.h"
#include "gtsam/geometry/Rot3.h"
#include "gtsam/inference/Symbol.h"
#include "gtsam/slam/BetweenFactor.h"
#include "gtsam/slam/PriorFactor.h"
#include "gtsam/nonlinear/NonlinearFactorGraph.h"
#include "gtsam/nonlinear/LevenbergMarquardtOptimizer.h"

#include <gtsam/slam/dataset.h>
#include <fstream>

using namespace std;
using namespace gtsam;

int main(const int argc, const char *argv[])
{
    double t_start = clock();
    vector<string> g2oFiles;

    g2oFiles.push_back(findExampleDataFile("pose3example.txt"));
    g2oFiles.push_back(findExampleDataFile("pose3example.txt"));
    g2oFiles.push_back(findExampleDataFile("pose3example.txt"));

    bool is3D = true;

    
    vector<NonlinearFactorGraph::shared_ptr> graphs;
    vector<Values::shared_ptr> initials;

    cout << "Time taken: " << (clock() - t_start) / CLOCKS_PER_SEC << "s" << endl;
    return 0;
}