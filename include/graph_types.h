#ifndef GRAPH_TYPES_H
#define GRAPH_TYPES_H

#include <gtsam/geometry/Pose2.h>
#include <gtsam/base/Matrix.h>

using vec3 = gtsam::Vector3;
using Pose2 = gtsam::Pose2;

struct Node {
    int id;
    Pose2 absolute_pose;

    Node(int id, const Pose2 &abs_pose)
    : id(id), absolute_pose(abs_pose) {}
};

// represents an edge from one pose to another
struct Edge {
    int x1, x2; // from, to
    Pose2 relative_pose; // relative_pose
    gtsam::Matrix33 cov; // covariance

    Edge(int x1, int x2, const Pose2 &relative_pose, const gtsam::Matrix33 &cov) 
    : x1(x1), x2(x2),
      relative_pose(relative_pose),
      cov(cov) {}
};

#endif
