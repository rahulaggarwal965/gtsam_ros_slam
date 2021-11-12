#ifndef SLAM_H
#define SLAM_H


#include <ros/ros.h>
#include "ros/subscriber.h"

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_msgs/TFMessage.h>
#include "tf/LinearMath/Quaternion.h"
#include "tf/exceptions.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_datatypes.h"
#include "tf/transform_listener.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <laser_geometry/laser_geometry.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/BearingRangeFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/linear/NoiseModel.h>

#include "graph_types.h"
#include "pose_utils.h"


using vec3 = gtsam::Vector3;
using Pose2 = gtsam::Pose2;
namespace NoiseModel = gtsam::noiseModel;

struct SLAM {

    // keeping track of most recently input sensor data
    sensor_msgs::LaserScan last_scan;

    // Markov chain corresponding to the state_estimator odometry
    Pose2 last_pose_odom{0, 0, 0};
    int idx = 0;

    // Discontinuous chain corresponding to scan matching 
    Pose2 last_pose_scan{0, 0, 0};
    int last_scan_index = 0;

    // for backend
    gtsam::ISAM2 isam;
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial_estimate;

    // ros basics
    ros::NodeHandle nh;

    // ros topics to subscribe to
    ros::Subscriber scan_sub; // /scan
    ros::Subscriber odom_mc_sub; // /odom_mc (also takes care of imu in a way)
    ros::Subscriber scan_match_sub; // /pose_with_covariance_stamped

    // publishers/broadcasters
    ros::Publisher path_pub; // /trajectory
    ros::Publisher pose_pub; // /pose
    ros::Publisher cloud_pub; // pointcloud
    tf::TransformBroadcaster transform_broadcaster; // map -> odom

    tf::TransformListener transform_listener;

    laser_geometry::LaserProjection projector;

    // rep of graph for saving later
    gtsam::Values current_poses;
    /* std::vector<Node> nodes; */
    std::vector<Node> scan_nodes;
    std::vector<Edge> edges;

    gtsam::Matrix33 min_markov_covariance;
    gtsam::Matrix33 min_scan_covariance;

    // TODO(rahul): add landmarks

    SLAM() {
        //                            topic                     queue size     function                 object       
        scan_sub       = nh.subscribe("scan",                         1, &SLAM::scan_callback,            this);
        odom_mc_sub    = nh.subscribe("odom_mc",                      1, &SLAM::state_estimator_callback, this);
        scan_match_sub = nh.subscribe("pose_with_covariance_stamped", 1, &SLAM::pose_callback,            this);

        path_pub  = nh.advertise<nav_msgs::Path>("trajectory", 1);
        pose_pub  = nh.advertise<geometry_msgs::PoseStamped>("pose", 1);
        cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("pointcloud", 1);

        min_markov_covariance << 1e-4, 1e-8, 1e-8,
                                 1e-8, 1e-4, 1e-8,
                                 1e-8, 1e-8, degrees_to_radians(10) * degrees_to_radians(10);

        min_scan_covariance  << 1e-4, 1e-8, 1e-8,
                                 1e-8, 1e-4, 1e-8,
                                 1e-8, 1e-8, degrees_to_radians(2) * degrees_to_radians(2);

        this->initial_estimate.insert(0, this->last_pose_odom);
        this->current_poses.insert(0, this->last_pose_odom);
        Pose2 prior_mean(0, 0, 0);
        NoiseModel::Diagonal::shared_ptr prior_noise = NoiseModel::Diagonal::Sigmas(vec3{0, 0, 0});
        this->graph.add(gtsam::PriorFactor<Pose2>(this->idx, prior_mean, prior_noise));
        
        // TODO(rahul): make ISAM2Params

    }


    void publish(ros::Time timestamp);

    void scan_callback(const sensor_msgs::LaserScan &scan);
    void state_estimator_callback(const nav_msgs::Odometry &state_estimated_odometry);
    void pose_callback(const geometry_msgs::PoseWithCovarianceStamped &pose_covariance_stamped);


    void optimize(int steps);
    void save(std::string file_name);
};

#endif
