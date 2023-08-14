import numpy as np
from graphviz import Digraph
import gtsam
import argparse
import open3d as o3d
from gtsam import (
    Pose3,
    Rot3,
    Point3,
    Values,
    NonlinearFactorGraph,
    PriorFactorPose3,
    BetweenFactorPose3,
    Symbol,
)


# GNC LM Optimizer
LM_params = gtsam.LevenbergMarquardtParams()
gnc_lm_params = gtsam.GncLMParams(LM_params)

print("Welcome to RRC DPGO")

Robot_Symbol = ["A", "B", "C", "D", "E", "F", "G", "H", "I", "J"]


class Robot:
    def __init__(self, robot_id, file_path):
        self.robot_id = robot_id
        self.file_path = file_path
        self.poses = []
        self.edges = []
        self.symbol = Robot_Symbol[robot_id]

    # getters and setters
    def set_symbol(self, symbol):
        self.symbol = symbol

    def set_path(self, file_path):
        self.file_path = file_path

    def set_poses(self, poses):
        self.poses = poses

    def set_edges(self, edges):
        self.edges = edges

    def set_robot_id(self, robot_id):
        self.robot_id = robot_id

    def get_file_path(self):
        return self.file_path

    def get_poses(self):
        return self.poses

    def get_edges(self):
        return self.edges

    def get_robot_id(self):
        return self.robot_id

    def get_symbol(self):
        return self.symbol
    
    def read_g2o_file(self):
        with open(self.file_path, "r") as f:
            for line in f:
                data = line.strip().split()
                if len(data) == 0:
                    continue
                if data[0] == "VERTEX_SE3:QUAT":  # Check if it's an SE3 vertex line
                    pose_id = int(data[1])
                    x, y, z = float(data[2]), float(data[3]), float(data[4])
                    qw, qx, qy, qz = (
                        float(data[5]),
                        float(data[6]),
                        float(data[7]),
                        float(data[8]),
                    )

                    # Save the SE3 pose as a 4x4 transformation matrix
                    # pose_matrix = np.array(
                    #     [
                    #         [
                    #             1 - 2 * qy**2 - 2 * qz**2,
                    #             2 * (qx * qy - qw * qz),
                    #             2 * (qx * qz + qw * qy),
                    #             x,
                    #         ],
                    #         [
                    #             2 * (qx * qy + qw * qz),
                    #             1 - 2 * qx**2 - 2 * qz**2,
                    #             2 * (qy * qz - qw * qx),
                    #             y,
                    #         ],
                    #         [
                    #             2 * (qx * qz - qw * qy),
                    #             2 * (qy * qz + qw * qx),
                    #             1 - 2 * qx**2 - 2 * qy**2,
                    #             z,
                    #         ],
                    #         [0, 0, 0, 1],
                    #     ]
                    # )

                    pose = Pose3(Rot3.Quaternion(qw, qx, qy, qz), Point3(x, y, z))

                    self.poses.append(pose)

                elif data[0] == "EDGE_SE3:QUAT":  # Check if it's an SE3 edge line
                    edge_id = int(data[1])
                    from_node_id = int(data[2])
                    to_node_id = int(data[3])
                    x, y, z = float(data[4]), float(data[5]), float(data[6])
                    qw, qx, qy, qz = (
                        float(data[7]),
                        float(data[8]),
                        float(data[9]),
                        float(data[10]),
                    )

                    # Save the SE3 edge as a 4x4 transformation matrix
                    # edge_matrix = np.array(
                    #     [
                    #         [
                    #             1 - 2 * qy**2 - 2 * qz**2,
                    #             2 * (qx * qy - qw * qz),
                    #             2 * (qx * qz + qw * qy),
                    #             x,
                    #         ],
                    #         [
                    #             2 * (qx * qy + qw * qz),
                    #             1 - 2 * qx**2 - 2 * qz**2,
                    #             2 * (qy * qz - qw * qx),
                    #             y,
                    #         ],
                    #         [
                    #             2 * (qx * qz - qw * qy),
                    #             2 * (qy * qz + qw * qx),
                    #             1 - 2 * qx**2 - 2 * qy**2,
                    #             z,
                    #         ],
                    #         [0, 0, 0, 1],
                    #     ]
                    # )

                    edge_pose = Pose3(Rot3.Quaternion(qw, qx, qy, qz), Point3(x, y, z))

                    self.edges.append((from_node_id, to_node_id, edge_pose))


# this is a private class, wont be using these functions directly so make them accordingly
class Submap:
    def __init__(self, submap_id):
        self.submap_id = submap_id

        self.robots = []
        self.nodes = []
        self.edges = []

        self.graph_viz = Digraph(comment="Submap Graph {}".format(submap_id))

        self.graph = NonlinearFactorGraph()

        self.initial = Values()
        self.optimized = Values()

        self.graph.add(PriorFactorPose3(0, Pose3(), 1e-6 * np.eye(6)))
        self.initial.insert(0, Pose3())

    def add_robot(self, robot):
        self.robots.append(robot)
        for i in range(len(robot.poses)):
            self.add_node(Symbol(robot.get_symbol(), i), robot.poses[i])
        for i in range(len(robot.edges)):
            self.add_edge(robot.edges[i][0], robot.edges[i][1], robot.edges[i][2])

    def add_node(self, node_id, pose):
        self.nodes.append(pose)
        # Edit node_id
        self.initial.insert(Symbol(self.robots[0].get_symbol(), node_id), Pose3(pose))

    def add_edge(self, from_node_id, to_node_id, edge_matrix):
        self.edges.append((from_node_id, to_node_id, edge_matrix))

    def print_submap(self):
        print(f"Submap ID: {self.submap_id}")
        print("Robots:")
        print(self.robots)
        # print("Vertices:")
        for pose in self.nodes:
            print(pose)
        print("Edges:")
        for edge in self.edges:
            print(edge)
        self.graph.print("Graph: \n")

    def optimize(self):
        pass


class Map:
    def __init__(self):
        self.submaps = {}

    def add_submap(self, submap_id):
        if submap_id not in self.submaps:
            self.submaps[submap_id] = Submap(submap_id)
        else:
            print(f"Submap {submap_id} already exists")

    def add_robot_to_submap(self, submap_id, robot):
        if submap_id not in self.submaps:
            print(f"Submap {submap_id} does not exist")
            return
        self.submaps[submap_id].add_robot(robot)

    def add_pose_to_submap(self, submap_id, pose):
        if submap_id not in self.submaps:
            print(f"Submap {submap_id} does not exist")
            return
        self.submaps[submap_id].add_node(pose)

    # merge map based on the different cliques found using connected sets
    def add_edge_to_submap(self, submap_id, from_node_id, to_node_id, edge):
        if submap_id not in self.submaps:
            print(f"Submap {submap_id} does not exist")
            return
        self.submaps[submap_id].add_edge(from_node_id, to_node_id, edge)

    # rewrtie this function
    def merge_submaps(self, robot1_id, robot2_id):
        submap1_id = f"submap_{robot1_id}"
        submap2_id = f"submap_{robot2_id}"

        # If submaps do not exist, create them
        if submap1_id not in self.submaps:
            self.add_submap(submap1_id)
        if submap2_id not in self.submaps:
            self.add_submap(submap2_id)

        submap1 = self.submaps[submap1_id]
        submap2 = self.submaps[submap2_id]

        combined_submap_id = f"submap_{robot1_id}_{robot2_id}"
        combined_submap = Submap(combined_submap_id)

        # Merge poses
        combined_submap.nodes = submap1.nodes + submap2.nodes

        # Merge edges
        combined_submap.edges = submap1.edges + submap2.edges

        # Add robots to the combined submap
        combined_submap.robots = list(set(submap1.robots + submap2.robots))

        # Remove individual submaps for the robots
        del self.submaps[submap1_id]
        del self.submaps[submap2_id]

        # Add the combined submap to the map
        self.submaps[combined_submap_id] = combined_submap

    def print_map(self):
        print("Map:")
        for submap_id, submap in self.submaps.items():
            submap.print_submap()


def main(args):
    map_obj = Map()
    Robots = []
    for i in range(args.num_files):
        file_name = args.file_names[i]
        print(f"Processing file {i+1}: {file_name}")
        robot = Robot(i, file_name)
        robot.read_g2o_file()
        Robots.append(robot)

    for robot in Robots:
        # After reading all the files, add poses and edges to the map
        submap_id = f"submap_{robot.robot_id}"
        map_obj.add_submap(submap_id)
        map_obj.add_robot_to_submap(submap_id, robot.robot_id)
        for pose in robot.poses:
            map_obj.add_pose_to_submap(submap_id, pose)
        for from_node_id, to_node_id, edge_pose in robot.edges:
            map_obj.add_edge_to_submap(submap_id, from_node_id, to_node_id, edge_pose)

    # After reading all the files and creating submaps for each robot, check for loop closures and merge submaps if needed
    # For example, assuming loop closures between robot 0 and robot 1:
    robot1_id = 0
    robot2_id = 1
    map_obj.merge_submaps(robot1_id, robot2_id)

    # Now you can access the combined submap for robots 0 and 1 by using their combined submap id, e.g., "submap_0_1"
    combined_submap_id = f"submap_{robot1_id}_{robot2_id}"
    combined_submap = map_obj.submaps[combined_submap_id]
    print(f"Combined Submap Poses: {combined_submap.nodes}")
    print(f"Combined Submap Edges: {combined_submap.edges}")

    # Print the entire map with all submaps, vertices, and edges
    map_obj.print_map()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Process multiple files")

    # Add arguments to the parser
    parser.add_argument("num_files", type=int, help="Number of files to be processed")
    parser.add_argument(
        "file_names", nargs="+", help="Names of the input files (separated by space)"
    )

    # Parse the command-line arguments
    args = parser.parse_args()

    # Call the main function with the parsed arguments
    main(args)
