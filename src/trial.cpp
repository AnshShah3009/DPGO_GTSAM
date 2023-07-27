#include <iostream>
#include "gtsam/geometry/Pose3.h"
#include "gtsam/geometry/Rot3.h"
#include "gtsam/inference/Symbol.h"
#include "gtsam/slam/BetweenFactor.h"
#include "gtsam/slam/PriorFactor.h"
#include "gtsam/nonlinear/NonlinearFactorGraph.h"
#include "gtsam/nonlinear/LevenbergMarquardtOptimizer.h"
#include <fstream>
#include <filesystem>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <mutex>
#include <thread>
// #include <ros/ros.h>

using namespace std;
using namespace gtsam;
namespace fs = std::filesystem;
string root = "/home/baymax/DPGO/data/";

// this will be the file where we input things.
// we will try to make it in a dynamic style
//  first argument is to add a file

// class to take inputs from the file and add it to one map

class Keyframes
{
public:
    string keyframe_id;
    Pose3 local_pose;
    Pose3 global_pose;
    string PointCloud;
    cv::Mat Keyframe;
    Keyframes(string id, Pose3 pose, string PointCloud)
    {
        this->keyframe_id = id;
        this->local_pose = pose;
        this->global_pose = pose;
        this->PointCloud = PointCloud;
    }
    Keyframes(string id, Pose3 pose, cv::Mat Keyframe)
    {
        this->keyframe_id = id;
        this->local_pose = pose;
        this->global_pose = pose;
        this->Keyframe = Keyframe;
    }
};

class Robot{
    public:
    int robot_id;
    string robot_name;
    string file_names;
    Robot(int id, string name){
        this->robot_id = id;
    }
    Robot(int id){
        this->robot_id = id;
        this->robot_name = robot_name + to_string(id);
    }
    void load_file(string file_g20){
        // keyframes.push_back(keyframe);
    }
};

class Map
{
public:
    int map_id;
    NonlinearFactorGraph graph;
    Values initial;
    Values result;
    vector<tuple<int, string, string>> loop_closures;
    int robot_count;
    int factor_count;
    vector<char> robot_name;
    string root = root;
    thread keyframe_add;
    mutex keyframelist_mutex;
    Map(vector<string> &file_names, vector<char> &robot_names)
    {
        // graph.add_factor();
        // map_id = robot_id;
        // robot_count = 0;
        // factor_count = 0;
        // robot_name = 'a';
        // add_to_graph();
    }
    // function to load the file
    // function to add file to NonLinear Factor Graph and Values by comapring already entered Values
    // function to optimize the graph
    // function send update poses into txt files
    //  fs::path update_file(string file_name){
    //      if (fs::is_regular_file(root / file_name))
    //          return (fs::path (root / file_name).string());
    //  }
    void add_to_graph()
    {
        // add the file to the graph
    }
};

int main(const int argc, const char *argv[])
{
    cv::Mat a;
    NonlinearFactorGraph graph;
    graph.add(PriorFactor<Pose3>(Symbol('x', 1), Pose3(Rot3::Identity(), Point3(0, 0, 0)), noiseModel::Unit::Create(6)));
    graph.add(PriorFactor<Pose3>(Symbol('x', 2), Pose3(Rot3::Identity(), Point3(2, 0, 0)), noiseModel::Unit::Create(6)));
    graph.add(BetweenFactor<Pose3>(Symbol('x', 1), Symbol('x', 2), Pose3(Rot3::Identity(), Point3(2, 0, 0)), noiseModel::Unit::Create(6)));
    Values initial;
    initial.insert(Symbol('x', 1), Pose3(Rot3::RzRyRx(0.2, 0.3, -0.1), Point3(0.5, 0.0, -0.2)));
    initial.insert(Symbol('x', 2), Pose3(Rot3::RzRyRx(0.1, 0.2, -0.3), Point3(1.8, 0.1, 0.2)));
    Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();
    result.print("Final result:\n");

    cout << "Hello World!" << endl;
    return 0;
}