#include "pose_utils.h"

PoseFactor generate_pose_factor(const geometry_msgs::PoseWithCovariance &pose_with_covariance, const Pose2 &pose_ref, const gtsam::Matrix33 &min_cov) {
    PoseFactor pf;
    float theta = tf::getYaw(pose_with_covariance.pose.orientation);
    pf.pose = create_pose2(pose_with_covariance.pose.position, theta);
    pf.relative_pose = compute_relative_pose(pose_ref, pf.pose);

    // we extract the [x, y, theta] covariances from the larger covariance matrix
    // [x y z roll pitch yaw]
    // z is only important for 3d slam, and roll/pitch are usually pretty accurate
    // with the internal state estimator
    // we also have a minimum covariance (0 noise not really possible)
    pf.cov =  min_cov.cwiseMax(extract_covariance_2d(pose_with_covariance.covariance));
    return pf;
}
