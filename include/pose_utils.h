#ifndef POSE_UTILS_H
#define POSE_UTILS_H

#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "tf/transform_datatypes.h"
#include <gtsam/geometry/Pose2.h>
#include <gtsam/base/Matrix.h>

using vec3 = gtsam::Vector3;
using Pose2 = gtsam::Pose2;

struct PoseFactor {
    Pose2 pose;
    Pose2 relative_pose;
    gtsam::Matrix33 cov;
};

inline float degrees_to_radians(float degrees) {
    return degrees / (180 * M_PI);
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

inline Pose2 compute_relative_pose(const Pose2 &a, const Pose2 b) {
    float x = b.x() - a.x();
    float y = b.y() - a.y();

    // account for changing orientation across coordinate frames
    float d_theta = b.theta() - a.theta();
    float d_x = cos(b.theta()) * x + sin(b.theta()) * y;
    float d_y = sin(-b.theta()) * x + cos(-b.theta()) * y;

    return { d_x, d_y, d_theta };

}

inline Pose2 normalize_pose_orientation(const Pose2 &a) {
    float new_theta = a.theta();
    if (new_theta > 2 * M_PI) {
        new_theta -= 2 * M_PI;
    } else if (new_theta < 2 * M_PI) {
        new_theta += 2 * M_PI;;
    }

    return { a.x(), a.y(), new_theta };
}

PoseFactor generate_pose_factor(const geometry_msgs::PoseWithCovariance &pose_with_covariance, const Pose2 &pose_ref, const gtsam::Matrix33 &min_cov);

#endif
