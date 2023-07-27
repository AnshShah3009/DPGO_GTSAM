#include <iostream>
#include "gtsam/geometry/Pose3.h"
#include "gtsam/geometry/Rot3.h"
#include "gtsam/inference/Symbol.h"
#include "gtsam/slam/BetweenFactor.h"
#include "gtsam/slam/PriorFactor.h"
#include "gtsam/nonlinear/NonlinearFactorGraph.h"
#include <gtsam/slam/dataset.h>
#include <fstream>
// #include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/GncOptimizer.h>
#include <gtsam/nonlinear/GncParams.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

using namespace std;
using namespace gtsam;

int main()
{
    // Create an empty factor graph
    NonlinearFactorGraph graph;
    Values initialEstimate;

    noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9).finished());
    noiseModel::Diagonal::shared_ptr MatchLinkNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-4, 1e-4, 1e-4, 1e-3, 1e-3, 1e-3).finished());
    noiseModel::Diagonal::shared_ptr LinkNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
    noiseModel::Diagonal::shared_ptr LinkNoise2 = noiseModel::Diagonal::Variances((Vector(6) << 1e-4, 1e-4, 1e-4, 1e-3, 1e-3, 1e-3).finished());

    vector<Pose3> poses;
    vector<Pose3> poses_ref;
    vector<pair<int, int>> edges;
    Pose3 pose1(Rot3::RzRyRx(0.0, 0.0, 0.0), Point3(0.0, 0.0, 0.0));
    Pose3 pose2(Rot3::RzRyRx(0.0, 0.0, 0.0), Point3(1.0, 1.0, 0.0));
    Pose3 pose3(Rot3::RzRyRx(0.0, 0.0, 0.0), Point3(2.0, 1.0, 0.0));
    Pose3 pose4(Rot3::RzRyRx(0.0, 0.0, 0.0), Point3(3.0, 0.0, 1.0));
    Pose3 pose5(Rot3::RzRyRx(0.0, 0.0, 0.0), Point3(4.0, 0.0, 1.0));
    Pose3 pose6(Rot3::RzRyRx(0.0, 0.0, 0.0), Point3(5.0, 1.0, -2.0));

    Pose3 deviate = Pose3(Rot3::RzRyRx(0.0, 0.0, 0.0), Point3(0.0, 0.0, 0.0));
    Pose3 Outlier = Pose3(Rot3::RzRyRx(1.0, 0.0, 0.0), Point3(0.0, 0.0, 0.0));

    // Pose3 pose7_1 = pose1.operator*(deviate);
    // Pose3 pose8_2 = pose2.operator*(deviate);
    // Pose3 pose9_3 = pose3.operator*(deviate);
    // Pose3 pose10_4 = pose4.operator*(deviate);
    // Pose3 pose11_5 = pose5.operator*(deviate);
    // Pose3 pose12_6 = pose6.operator*(deviate);

    Pose3 pose7_1 = Pose3(Rot3(1, 0, 0, 0, 1, 0, 0, 0, 1), Point3(105.0, 0.0, 0.0));
    Pose3 pose8_2 = Pose3(Rot3(1, 0, 0, 0, 1, 0, 0, 0, 1), Point3(101.0, 1.0, 0.0));
    Pose3 pose9_3 = Pose3(Rot3(1, 0, 0, 0, 1, 0, 0, 0, 1), Point3(102.0, 1.0, 0.0));
    Pose3 pose10_4 = Pose3(Rot3(1, 0, 0, 0, 1, 0, 0, 0, 1), Point3(103.0, 0.0, 1.0));
    Pose3 pose11_5 = Pose3(Rot3(1, 0, 0, 0, 1, 0, 0, 0, 1), Point3(104.0, 0.0, 1.0));
    Pose3 pose12_6 = Pose3(Rot3(1, 0, 0, 0, 1, 0, 0, 0, 1), Point3(105.0, 1.0, -2.0));

    poses.push_back(pose1);
    poses.push_back(pose2);
    poses.push_back(pose3);
    poses.push_back(pose4);
    poses.push_back(pose5);
    poses.push_back(pose6);

    poses_ref.push_back(pose7_1);
    poses_ref.push_back(pose8_2);
    poses_ref.push_back(pose9_3);
    poses_ref.push_back(pose10_4);
    poses_ref.push_back(pose11_5);
    poses_ref.push_back(pose12_6);

    graph.addPrior(0, poses.at(0), priorNoise);

    for (int i = 0; i < poses.size(); ++i)
    {
        initialEstimate.insert(i, poses.at(i));
        for (int j = i; j < poses.size(); ++j)
        {
            if (i != j)
            {
                edges.push_back(make_pair(i, j));
                graph.add(BetweenFactor<Pose3>(i, j, poses.at(i).between(poses.at(j)), LinkNoise));
            }
        }
        // cout << << poses.at(i) << endl;
    }

    for (int i = 0; i < poses_ref.size(); ++i)
    {
        initialEstimate.insert(i + 10, Pose3());
        for (int j = i; j < poses_ref.size(); ++j)
        {
            if (i != j)
            {
                edges.push_back(make_pair(i + 10, j + 10));
                graph.add(BetweenFactor<Pose3>(i + 10, j + 10, poses_ref.at(i).between(poses_ref.at(j)), LinkNoise2));

                // comment this line to see the difference between the two cases (with and without complete interconnection)
                // edges.push_back(make_pair(i + 10, j));
                // graph.add(BetweenFactor<Pose3>(i + 10, j, poses.at(i).between(poses.at(j)), MatchLinkNoise));
                // cout << "(" << i << "," << j << ")" << poses_ref.at(i).between(poses_ref.at(j)) << endl;
            }
        }
        // cout << << poses.at(i) << endl;
    }

    for (int i = 0; i < poses.size(); ++i)
    {
        edges.push_back(make_pair(i, i + 10));
        graph.add(BetweenFactor<Pose3>(i, i + 10, Pose3(), MatchLinkNoise));
    }

    LevenbergMarquardtParams params;
    params.setVerbosity("ERROR");     // Set verbosity level to "ERROR" to suppress unnecessary output
    params.setRelativeErrorTol(1e-5); // Set the relative error tolerance for convergence
    params.setMaxIterations(1000);

    // GTSAM_CONCEPT_GROUP_TYPE
    GncParams parameters = GncParams(params);
    parameters.lossType = gtsam::GncLossType::TLS;

    // KeyVector Inliers;
    // Inliers.emplace_back(0);
    // Inliers.emplace_back(1);
    // Inliers.emplace_back(2);
    // Inliers.emplace_back(3);
    // Inliers.emplace_back(4);
    // Inliers.emplace_back(5);
    // Inliers.emplace_back(10);
    // Inliers.emplace_back(11);
    // Inliers.emplace_back(12);
    // Inliers.emplace_back(13);
    // Inliers.emplace_back(14);
    // Inliers.emplace_back(15);
    // parameters.setKnownInliers(Inliers);

    // KeyVector Outliers;
    // Outliers.emplace_back(key2);
    // parameters.setKnownOutliers(Outliers);

    // Create the optimization algorithm
    GncOptimizer optimizer(graph, initialEstimate, parameters);

    // // Perform the optimization
    Values optimizedEstimate = optimizer.optimize();
    // initialEstimate.print("Initial Estimate:\n");
    optimizedEstimate.print("Final Result:\n");
    // graph.print("Final Graph:\n");

    Vector weigths = optimizer.getWeights();

    for (int i = 1; i < weigths.size(); ++i)
    {
        cout << "Edge : " << edges.at(i - 1).first << "," << edges.at(i - 1).second << endl;
        cout << "Static Probablity : " << weigths(i) << endl;
    }
    return 0;
}
