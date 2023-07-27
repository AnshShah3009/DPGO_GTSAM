#include <iostream>
#include "gtsam/geometry/Pose3.h"
#include "gtsam/geometry/Rot3.h"
#include "gtsam/inference/Symbol.h"
#include "gtsam/slam/BetweenFactor.h"
#include "gtsam/slam/PriorFactor.h"
#include "gtsam/nonlinear/NonlinearFactorGraph.h"

#include <gtsam/slam/dataset.h>
#include <fstream>

#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/GncOptimizer.h>
#include <gtsam/nonlinear/GncParams.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

using namespace std;
using namespace gtsam;

// bool ReadVertex(std::ifstream *infile,
//                 std::map<int, Pose, std::less<int>, Allocator> *poses)
// {
//     int id;
//     gtsam::axpy pose;
//     *infile >> id >> pose;
//     // Ensure we don't have duplicate poses.
//     if (poses->find(id) != poses->end())
//     {
//         // LOG(RROR) << "Duplicate vertex with ID: " << id;
//         return false;
//     }
//     (*poses)[id] = pose;
//     return true;
// }

// TODO
//  Create a joint file for g2o to add a infinite invariance
//  Create a vector of filenames
//  After adding infinite prior, add the other files
//  Create a function to read the g2o file
//  Create a function to trajectory to ROS messages

// Reads the contraints between two vertices in the pose graph
// template <typename Constraint, typename Allocator>
// void ReadConstraint(std::ifstream *infile,
//                     std::vector<Constraint, Allocator> *constraints)
// {
//     Constraint constraint;
//     *infile >> constraint;
//     constraints->push_back(constraint);
// }

// void readG2o()
// {
//     while (infile.good())
//     {
//         // Read whether the type is a node or a constraint.
//         infile >> data_type;
//         if (data_type == Pose::name())
//         {
//             if (!ReadVertex(&infile, poses))
//             {
//                 return false;
//             }
//         }
//         else if (data_type == Constraint::name())
//         {
//             ReadConstraint(&infile, constraints);
//         }
//         else
//         {
//             // LOG(ERROR) << "Unknown data type: " << data_type;
//             return false;
//         }
//     }

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
    // graph.add(BetweenFactor<Pose2>(key6, key1, Pose2(0.0, 0.0, 0.0), priorNoise));

    // graph.add(<BetweenFactor<Pose2>>(key5, key1, Pose2(1.0, 0.0, 0.0), priorNoise));
    cout << "Baymax" << key1 << "___" << key2 << endl;
    // Create an initial estimate for the variables
    Values initialEstimate;
    initialEstimate.insert(key1, Pose2(0.5, 0.5, 0.1));
    initialEstimate.insert(key2, Pose2(1.5, 0.3, -0.2));
    initialEstimate.insert(key3, Pose2(0.5, 0.5, 0.1));
    initialEstimate.insert(key4, Pose2(1.5, 0.3, -0.2));
    initialEstimate.insert(key5, Pose2(1.5, 0.3, -0.2));
    initialEstimate.insert(key6, Pose2(1.5, 0.3, -0.2));

    // initialEstimate.insert(key5, Pose2(0.5, 0.5, 0.1));

    // Create the optimization parameters with outlier rejection enabled
    LevenbergMarquardtParams params;
    params.setVerbosity("COMPLEXITY"); // Set verbosity level to "ERROR" to suppress unnecessary output
    params.setRelativeErrorTol(1e-5);  // Set the relative error tolerance for convergence
    params.setMaxIterations(1000);

    // GTSAM_CONCEPT_GROUP_TYPE
    GncParams parameters = GncParams(params);
    parameters.lossType = gtsam::GncLossType::TLS;

    cout << "baymax : keys()" << endl;

    for (auto it = graph.keys().begin(); it != graph.keys().end(); it++)
    {
        cout << "\t" << *it << endl;
    }
    cout << endl;

    cout << "baymax : keyVector()" << endl;

    for (auto it = graph.keyVector().begin(); it != graph.keyVector().end(); it++)
    {
        cout << "\t" << *it << endl;
    }
    cout << endl;

    // cout <<"baymax : "<< graph.keyVector().<< endl;

    // KeyVector Inliers;
    // Inliers.emplace_back(0);
    // Inliers.emplace_back(4);
    // Inliers.emplace_back(key2);
    // Inliers.emplace_back(key3);
    // Inliers.emplace_back(key4);
    // Inliers.emplace_back(key5);

    cout << "baymax :" << graph.keys().exists(key1) << ", " << graph.keys().exists(1) << endl;
    // int i = 0;
    // for (auto it = graph.keys().begin(); it != graph.keys().end(); it++){
    //     // cout<<"\t"<<*it<<endl;
    //     if (/*i == 0 ||*/ i == 2)
    //     Inliers.emplace_back(*it);

    //     i++;
    // }

    // parameters.setKnownInliers(Inliers);

    // KeyVector Outliers;
    // Outliers.emplace_back(key2);
    // parameters.setKnownOutliers(Outliers);

    // Create the optimization algorithm
    GncOptimizer optimizer(graph, initialEstimate, parameters);

    // Perform the optimization
    Values optimizedEstimate = optimizer.optimize();
    optimizedEstimate.print("Final Result:\n");
    graph.print("Final Graph:\n");
    cout << optimizer.getWeights() << endl;

    // cout << optimizer.getFactors().at(0) << endl;

    return 0;
}
