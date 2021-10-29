#include <cmath>
#include <ros/ros.h>
#include "ros/subscriber.h"

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_msgs/TFMessage.h>
#include "tf/LinearMath/Matrix3x3.h"
#include "tf/LinearMath/Quaternion.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_datatypes.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <eigen3/Eigen/Dense>

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/BearingRangeFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/linear/NoiseModel.h>

#include <nlohmann/json.hpp>

using json = nlohmann::json;

using vec3 = gtsam::Vector3;
using Pose2 = gtsam::Pose2;
namespace NoiseModel = gtsam::noiseModel;


Pose2 compute_relative_pose(const Pose2 &a, const Pose2 b) {
    float x = b.x() - a.x();
    float y = b.y() - a.y();

    // account for changing orientation across coordinate frames
    float d_theta = b.theta() - a.theta();
    float d_x =  cos(a.theta()) * x + sin(a.theta()) * y;
    float d_y = -sin(a.theta()) * x + cos(a.theta()) * y;

    return { d_x, d_y, d_theta };

}

Pose2 normalize_pose_orientation(const Pose2 &a) {
    float new_theta = a.theta();
    if (new_theta > 2 * M_PI) {
        new_theta -= 2 * M_PI;
    } else if (new_theta < 2 * M_PI) {
        new_theta += 2 * M_PI;;
    }

    return { a.x(), a.y(), new_theta };
}

// simple struct for returning data used by multiple callbacks
// odometry and scan_matching
struct PoseFactor {
    Pose2 pose;
    Pose2 relative_pose;
    gtsam::Matrix33 cov;
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

/* void to_json(json &j, const Pose2 &pose) { */
/*     j = json::array({pose.x(), pose.y(), pose.theta() }); */
/* } */
/*  */
/* void from_json(const json& j, Pose2& p) { */
/* } */
/*  */
/* void to_json(json &j, const Edge &edge) { */
/*     json pose = edge.relative_pose; */
/*     j = json{{"from", edge.x1}, {"to", edge.x2}, {"pose", pose}}; */
/* } */

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
    tf::TransformBroadcaster transform_broadcaster; // map -> odom

    // rep of graph for saving later
    gtsam::Values current_poses;
    std::vector<Edge> edges;

    // TODO(rahul): add landmarks

    SLAM() {
        //                            topic                     queue size     function                 object       
        scan_sub       = nh.subscribe("scan",                         1, &SLAM::scan_callback,            this);
        odom_mc_sub    = nh.subscribe("odom_mc",                      1, &SLAM::state_estimator_callback, this);
        scan_match_sub = nh.subscribe("pose_with_covariance_stamped", 1, &SLAM::pose_callback,            this);

        path_pub = nh.advertise<nav_msgs::Path>("trajectory", 1);
        pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 1);

        this->initial_estimate.insert(0, this->last_pose_odom);
        this->current_poses.insert(0, this->last_pose_odom);
        Pose2 prior_mean(0, 0, 0);
        NoiseModel::Diagonal::shared_ptr prior_noise = NoiseModel::Diagonal::Sigmas(vec3{0, 0, 0});
        this->graph.add(gtsam::PriorFactor<Pose2>(this->idx, prior_mean, prior_noise));
        
        // TODO(rahul): make ISAM2Params

    }

    void publish(ros::Time timestamp) {
        nav_msgs::Path path{};
        geometry_msgs::PoseStamped last_pose;

        path.header.frame_id = "map";

        for (const auto &p : this->current_poses) {
            Pose2 pose = p.value.cast<Pose2>();
            last_pose.header.frame_id = "map";
            last_pose.header.stamp = timestamp;
            last_pose.pose.position.x = pose.x();
            last_pose.pose.position.y = pose.y();
            last_pose.pose.position.z = 0;
            last_pose.pose.orientation = tf::createQuaternionMsgFromYaw(pose.theta());

            path.poses.push_back(last_pose);
        }

        this->path_pub.publish(path);
        if (this->current_poses.empty()) return;

        this->pose_pub.publish(last_pose);

    }

    void scan_callback(const sensor_msgs::LaserScan &scan) {
        this->last_scan = scan;
    }

    void state_estimator_callback(const nav_msgs::Odometry &state_estimated_odometry);
    /* void imu_callback(const sensor_msgs::Imu &imu_data); */
    void pose_callback(const geometry_msgs::PoseWithCovarianceStamped &pose_covariance_stamped);

    void optimize(int steps) {
        this->isam.update(this->graph, this->initial_estimate);
        for (int i = 0; i < steps; i++) {
            this->isam.update();
        }

        this->current_poses = this->isam.calculateEstimate();

        // reinitialize, ideally, we would not have to free this memory
        this->graph = gtsam::NonlinearFactorGraph();
        this->initial_estimate.clear();
    }

    void save(std::string file_name) {
        this->isam.saveGraph(file_name);
    }
};

int main(int argc, char **argv) {

    ros::init(argc, argv, "incremental_slam");
    SLAM slam;

    ros::spin();

    slam.save("/home/infinity/Code/ros/graph.gv");

    return 0;

}

inline Pose2 create_pose2(const geometry_msgs::Point &p, float theta) {
    return { p.x, p.y, theta };
}

inline float mag(const Pose2 &pose) {
    return sqrt(pose.x() * pose.x() + pose.y() * pose.y());
}

inline gtsam::Matrix33 extract_covariance_2d(const boost::array<double, 36> &cov) {
    gtsam::Matrix33 r;
    r <<  cov[0],  cov[1],  cov[5],     //        xx     xy     xtheta
          cov[6],  cov[7],  cov[11],    //        xy     yy     ytheta
          cov[30], cov[31], cov[35];    //    xtheta ytheta thetatheta
    return r;
}

PoseFactor generate_pose_factor(const geometry_msgs::PoseWithCovariance &pose_with_covariance, const Pose2 &pose_ref) {
    PoseFactor pf;
    float theta = tf::getYaw(pose_with_covariance.pose.orientation);
    pf.pose = create_pose2(pose_with_covariance.pose.position, theta);
    pf.relative_pose = compute_relative_pose(pose_ref, pf.pose);

    // we extract the [x, y, theta] covariances from the larger covariance matrix
    // [x y z roll pitch yaw]
    // z is only important for 3d slam, and roll/pitch are usually pretty accurate
    // with the internal state estimator
    pf.cov =  extract_covariance_2d(pose_with_covariance.covariance);
    return pf;
}

// this data will come in with much higher frequency compared to scans
void SLAM::state_estimator_callback(const nav_msgs::Odometry &state_estimated_odometry) {

    PoseFactor pf = generate_pose_factor(state_estimated_odometry.pose, this->last_pose_odom);
    this->last_pose_odom = pf.pose;

    NoiseModel::Diagonal::shared_ptr noise = NoiseModel::Diagonal::Sigmas(pf.cov.diagonal());

    // NOTE(rahul): we might have to use a mutex depending on how callbacks get priority
    auto normalized_pose = normalize_pose_orientation(pf.pose);
    auto normalized_relative_pose = normalize_pose_orientation(pf.relative_pose);

    // keep the pose in the odom frame as an initial estimate
    this->initial_estimate.insert(idx + 1, normalized_pose);
    this->current_poses.insert(idx + 1, normalized_pose);
     
    // now we insert into the actual graph
    this->graph.add(gtsam::BetweenFactor<Pose2>(idx, idx + 1, normalized_relative_pose, noise));
    this->edges.emplace_back(idx, idx + 1, normalized_relative_pose, pf.cov);

    idx++;

    this->optimize(10); // 10 is quite arbitary
    this->publish(state_estimated_odometry.header.stamp);    
}

// this data will come every time a new lidar scan comes into play
void SLAM::pose_callback(const geometry_msgs::PoseWithCovarianceStamped &pose_covariance_stamped) {

    if (this->last_scan_index == idx) return;

    PoseFactor pf = generate_pose_factor(pose_covariance_stamped.pose, last_pose_scan);
     
    auto normalized_relative_pose = normalize_pose_orientation(pf.relative_pose);

    // NOTE(rahul): should we put a minimum on the noise, because it's very unlikely to have 0 noise
    NoiseModel::Diagonal::shared_ptr noise = NoiseModel::Diagonal::Sigmas(pf.cov.diagonal());

    // we don't use these as an initial estimate
    this->graph.add(gtsam::BetweenFactor<Pose2>(last_scan_index, idx, normalized_relative_pose, noise));
    this->edges.emplace_back(last_scan_index, idx, normalized_relative_pose, pf.cov);
    last_scan_index = idx;

}
