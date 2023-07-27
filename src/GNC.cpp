#include <iostream>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include "gtsam/inference/Symbol.h"
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/GncOptimizer.h>
#include <gtsam/nonlinear/GncParams.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

using namespace std;
using namespace gtsam;

int main()
{
    // Create an empty factor graph
    NonlinearFactorGraph graph;

    // Create a noise model for the prior factor
    noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas(Vector3(0.1, 0.1, 0.1));

    // Add prior factors to the factor graph
    Key key1 = symbol('x', 1);
    graph.add(PriorFactor<Pose2>(key1, Pose2(0.0, 0.0, 0.0), priorNoise));

    Key key2 = symbol('x', 2);
    graph.add(BetweenFactor<Pose2>(key1, key2, Pose2(1.0, 0.0, 0.0), priorNoise));

    Key key3 = symbol('x', 3);
    graph.add(BetweenFactor<Pose2>(key2, key3, Pose2(1.0, 0.0, 0.0), priorNoise));

    Key key4 = symbol('x', 4);
    graph.add(BetweenFactor<Pose2>(key3, key4, Pose2(1.0, 0.0, 0.0), priorNoise));
    graph.add(BetweenFactor<Pose2>(key3, key4, Pose2(20.0, 0.0, 0.0), priorNoise)); // outlier
    graph.add(BetweenFactor<Pose2>(key3, key4, Pose2(1.1, 0.0, 0.0), priorNoise));
    graph.add(BetweenFactor<Pose2>(key4, key1, Pose2(-100.0, 1.0, 0.0), priorNoise)); // outlier

    Key key5 = symbol('x', 5);
    Key key6 = symbol('x', 6);
    graph.add(BetweenFactor<Pose2>(key5, key6, Pose2(1.0, 0.0, 0.0), priorNoise));
    graph.add(BetweenFactor<Pose2>(key6, key1, Pose2(0.0, 0.0, 0.0), priorNoise));

    // Create an initial estimate for the variables
    Values initialEstimate;
    initialEstimate.insert(key1, Pose2(0.5, 0.5, 0.1));
    initialEstimate.insert(key2, Pose2(1.5, 0.3, -0.2));
    initialEstimate.insert(key3, Pose2(0.5, 0.5, 0.1));
    initialEstimate.insert(key4, Pose2(1.5, 0.3, -0.2));
    initialEstimate.insert(key5, Pose2(1.5, 0.3, -0.2));
    initialEstimate.insert(key6, Pose2(1.5, 0.3, -0.2));

    // Create the optimization parameters
    LevenbergMarquardtParams params;
    params.setVerbosity("COMPLEXITY"); // Set verbosity level to "ERROR" to suppress unnecessary output
    params.setRelativeErrorTol(1e-5);  // Set the relative error tolerance for convergence
    params.setMaxIterations(1000);

    // Create the GNC optimizer parameters
    GncParams gncParams(params);
    gncParams.lossType = gtsam::GncLossType::TLS;

    // Create the GNC optimizer
    GncOptimizer optimizer(graph, initialEstimate, gncParams);

    // Perform the optimization
    Values optimizedEstimate = optimizer.optimize();
    optimizedEstimate.print("Final Result:\n");
    graph.print("Final Graph:\n");
    cout << optimizer.getWeights() << endl;

    return 0;
}
