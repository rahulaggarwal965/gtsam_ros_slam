#include <fstream>
#include <cmath>

#include "slam.h"
#include "pose_utils.h"

#include <nlohmann/json.hpp>

using json = nlohmann::json;

using vec3 = gtsam::Vector3;
using Pose2 = gtsam::Pose2;
namespace NoiseModel = gtsam::noiseModel;

void SLAM::publish(ros::Time timestamp) {
        nav_msgs::Path path{};
        geometry_msgs::PoseStamped last_pose;

        path.header.frame_id = "odom_mc";

        for (const auto &p : this->current_poses) {
            Pose2 pose = p.value.cast<Pose2>();
            last_pose.header.frame_id = "odom_mc";
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

void SLAM::optimize(int steps) {
        try {
            this->isam.update(this->graph, this->initial_estimate);
            for (int i = 0; i < steps; i++) {
                this->isam.update();
            }

            this->current_poses = this->isam.calculateEstimate();

            // reinitialize, ideally, we would not have to free this memory
            this->graph = gtsam::NonlinearFactorGraph();
            this->initial_estimate.clear();
        } catch(gtsam::IndeterminantLinearSystemException &e) {
            ROS_ERROR("%s\n", e.what());
            this->save("/home/infinity/Code/ros/graph.json");
        }
    }

void SLAM::save(std::string file_name) {
        /* this->isam.saveGraph(file_name); */
        json j;
        j["nodes"] = json::array();
        j["edges"] = json::array();
        j["scan_nodes"] = json::array();
        for (const auto &pose_identified : this->current_poses) {
            const auto &pose = pose_identified.value.cast<Pose2>();
            j["nodes"].push_back(
                {
                    {"id", pose_identified.key},
                    {"pose", {pose.x(), pose.y(), pose.theta()}}
                });
        }
        for (const auto &edge : this->edges) {
            j["edges"].push_back(
                {
                    {"from", edge.x1},
                    {"to", edge.x2},
                    {"relative_pose", {edge.relative_pose.x(), edge.relative_pose.y(), edge.relative_pose.theta()}},
                    {"type", edge.type},
                });
        }
        for (const auto &scan_node : this->scan_nodes) {
            const auto &pose = scan_node.absolute_pose;
            j["scan_nodes"].push_back(
                {
                    {"id", scan_node.id},
                    {"pose", {pose.x(), pose.y(), pose.theta()}}
                });
        }
        std::ofstream file(file_name);
        if (file.is_open()) {
            file << std::setw(4) << j << std::endl;
            file.close();
        } else {
            fprintf(stderr, "ERR: Could not write graph to file [%s]\n", file_name.c_str());
        }
        printf("Saving graph complete.\n");
    }

int main(int argc, char **argv) {

    ros::init(argc, argv, "incremental_slam");
    SLAM slam;

    ros::Rate r(30);

    while (ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }

    /* ros::spin(); */

    printf("Saving Graph .... \n");
    slam.save("/home/infinity/Code/ros/graph.json");

    return 0;

}


void SLAM::scan_callback(const sensor_msgs::LaserScan &scan) {
    this->last_scan = scan;
    sensor_msgs::PointCloud2 cloud;
    this->projector.projectLaser(scan, cloud);
    this->cloud_pub.publish(cloud); 
}

// this data will come in with much higher frequency compared to scans
void SLAM::state_estimator_callback(const nav_msgs::Odometry &state_estimated_odometry) {

    // transform the minicheetah odometry into the "odom"
    geometry_msgs::PoseStamped pose_in; 
    pose_in.pose = state_estimated_odometry.pose.pose;
    pose_in.header = state_estimated_odometry.header;

    geometry_msgs::PoseStamped transformed_pose;
    geometry_msgs::PoseWithCovariance transformed_pose_with_covariance;

    // NOTE(rahul): we HAVE to wrap this in a try/catch block otherwise, it breaks
    try {
        this->transform_listener.transformPose("odom_mc", pose_in, transformed_pose);
    } catch(tf::TransformException& e) {
        ROS_ERROR("%s", e.what());
        return;
    }
    transformed_pose_with_covariance.pose = transformed_pose.pose;
    transformed_pose_with_covariance.covariance = state_estimated_odometry.pose.covariance;

    /* PoseFactor pf = generate_pose_factor(state_estimated_odometry.pose, this->last_pose_odom); */
    PoseFactor pf = generate_pose_factor(transformed_pose_with_covariance, this->last_pose_odom, this->min_markov_covariance);
    this->last_pose_odom = pf.pose;

    NoiseModel::Diagonal::shared_ptr noise = NoiseModel::Diagonal::Sigmas(pf.cov.diagonal());

    // NOTE(rahul): we might have to use a mutex depending on how callbacks get priority
    auto normalized_pose = normalize_pose_orientation(pf.pose);
    auto normalized_relative_pose = normalize_pose_orientation(pf.relative_pose);

    // keep the pose in the odom frame as an initial estimate
    printf("Adding pose at idx[%d], pose=[%f, %f, %f]\n\n", 
            idx + 1,
            normalized_pose.x(),
            normalized_pose.y(),
            normalized_pose.theta());
    this->initial_estimate.insert(idx + 1, normalized_pose);
    this->current_poses.insert(idx + 1, normalized_pose);
     
    // now we insert into the actual graph
    printf("Adding edge between idx[%d] and idx[%d], rel_pose=[%f, %f, %f]\n\n", 
            idx,
            idx + 1,
            normalized_relative_pose.x(),
            normalized_relative_pose.y(),
            normalized_relative_pose.theta());
    this->graph.add(gtsam::BetweenFactor<Pose2>(idx, idx + 1, normalized_relative_pose, noise));
    this->edges.emplace_back(idx, idx + 1, normalized_relative_pose, ODOM);

    idx++;

    this->optimize(10); // 10 is quite arbitary
    this->publish(state_estimated_odometry.header.stamp);    
}

// this data will come every time a new lidar scan comes into play
void SLAM::pose_callback(const geometry_msgs::PoseWithCovarianceStamped &pose_covariance_stamped) {
    if (this->last_scan_index == idx) return;


    geometry_msgs::PoseStamped pose_in;
    pose_in.pose = pose_covariance_stamped.pose.pose;
    pose_in.header = pose_covariance_stamped.header;

    geometry_msgs::PoseStamped pose_out;
    geometry_msgs::PoseWithCovariance transformed_pose_with_covariance;

    try {
        this->transform_listener.transformPose("odom_mc", pose_in, pose_out);
    } catch(tf::TransformException &e) {
        ROS_ERROR("%s", e.what());
        return;
    }

    transformed_pose_with_covariance.pose = pose_out.pose;
    transformed_pose_with_covariance.covariance = pose_covariance_stamped.pose.covariance;

    printf("---------------- SCAN POSE ----------------\n");
    printf("scan_pose=[x=%f, y=%f, theta=%f]\n", 
        transformed_pose_with_covariance.pose.position.x,
        transformed_pose_with_covariance.pose.position.y,
        tf::getYaw(pose_covariance_stamped.pose.pose.orientation));
    printf("---------------- CURRENT MARKOV POSE ----------------\n");
    const auto &markov_pose = this->current_poses.at(idx).cast<Pose2>();
    printf("pose=[x=%f, y=%f, theta=%f]\n", 
        markov_pose.x(),
        markov_pose.y(),
        markov_pose.theta());
    printf("\n\n\n\n");
    

    PoseFactor pf = generate_pose_factor(transformed_pose_with_covariance, last_pose_scan, this->min_scan_covariance);
    this->last_pose_scan = pf.pose;
    /* PoseFactor pf = generate_pose_factor(pose_covariance_stamped.pose, last_pose_scan); */
     
    Pose2 p = create_pose2(
        transformed_pose_with_covariance.pose.position,
        tf::getYaw(transformed_pose_with_covariance.pose.orientation));
    scan_nodes.emplace_back(idx, p);
    auto normalized_relative_pose = normalize_pose_orientation(pf.relative_pose);

    // NOTE(rahul): should we put a minimum on the noise, because it's very unlikely to have 0 noise
    NoiseModel::Diagonal::shared_ptr noise = NoiseModel::Diagonal::Sigmas(pf.cov.diagonal());

    // we don't use these as an initial estimate
    ROS_INFO("Scan edge from %d to %d", last_scan_index, idx);
    this->graph.add(gtsam::BetweenFactor<Pose2>(last_scan_index, idx, normalized_relative_pose, noise));
    this->edges.emplace_back(last_scan_index, idx, normalized_relative_pose, SCAN);
    last_scan_index = idx;

}
