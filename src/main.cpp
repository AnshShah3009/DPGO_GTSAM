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
    string g2oFile;
    if (argc < 2)
        g2oFile = findExampleDataFile("pose3example.txt");
    else
        g2oFile = argv[1];
    
    bool is3D = true;
    NonlinearFactorGraph::shared_ptr graph_main;
    Values::shared_ptr initial_main;

    NonlinearFactorGraph::shared_ptr graph_1;
    Values::shared_ptr initial_1;
    
    NonlinearFactorGraph::shared_ptr graph_2;
    Values::shared_ptr initial_2;

    std::tie(graph_1, initial_1) = readG2o(g2oFile, is3D);
    std::tie(graph_2, initial_2) = readG2o(g2oFile, is3D);

    noiseModel::Diagonal::shared_ptr priorModel = noiseModel::Diagonal::Variances((Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
    noiseModel::Diagonal::shared_ptr infiModel = noiseModel::Isotropic::Sigma(6, 1e9);
    // gtsam::Matrix largeCovariance = Matrix::Constant(6, 6, 1e9); // Large covariance values
    // cout << largeCovariance << endl;
    noiseModel::Diagonal::shared_ptr infiniteNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e9, 1e9, 1e9, 1e9, 1e9, 1e9).finished());
    Key firstKey = 0;
    Symbol firstSym('x', firstKey);
    for (const auto key : graph_1->keys())
    {
        cout << "Adding prior to g2o file " << std::endl;
        cout << *graph_1.get()->keys().begin() <<" "<< *initial_1.get()->keys().begin() << endl;
        // firstKey = key;
        // graph->addPrior(firstKey, Pose3(), priorModel);
        // break;
    }
    graph_1->print();
    // for (const auto key : initial->keys())
    // {
    //     cout << "Adding prior to g2o file " << std::endl;
    //     firstKey = key;
    //     graph->addPrior(firstKey, Pose3(), priorModel);
    //     break;
    // }

    // Optimize the initial values using a Gauss-Newton nonlinear optimizer
    Values result = LevenbergMarquardtOptimizer(*graph_main, *initial_main).optimize();
    // result.print("Final result:\n");

    // if no output file is specified, print to screen
    if (argc < 3)
    {
        // result.print("result");
    }
    else
    {
        const string outputFile = argv[2];
        std::cout << "Writing results to file: " << outputFile << std::endl;
        NonlinearFactorGraph::shared_ptr graphNoKernel;
        Values::shared_ptr initial2;
        std::tie(graphNoKernel, initial2) = readG2o(g2oFile);
        writeG2o(*graphNoKernel, result, outputFile);
        std::cout << "done! " << std::endl;
    }
    return 0;
}