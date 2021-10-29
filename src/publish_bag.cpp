#include <ros/ros.h>
#include <ros/time.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/PoseStamped.h>
#include <laser_geometry/laser_geometry.h>

/* ------- TOPICS  -------
/clock                      : rosgraph_msgs/Clock          = timestamps
/cmd_vel                    : geometry_msgs/Twist          = velocity commands given to a1
/map                        : nav_msgs/OccupancyGrid       = occupancy grid of environment
/odom_mc                    : nav_msgs/Odometry            = raw odom data from minicheetah controller
/qualisys/a1_gazebo/odom    : nav_msgs/Odometry            = ground truth odom
/qualisys/a1_gazebo/pose    : geometry_msgs/PoseStamped    = ground truth pose
/scan                       : sensor_msgs/LaserScan        = lidar depths
/tf                         : tf2_msgs/TFMessage           = coordinate transformst that change over time
/tf_static                  : tf2_msgs/TFMessage           = coordinate transforms that are static with time
/trunk_imu                  : sensor_msgs/Imu              = IMU data from trunk of quadruped
*/

#define BAG_FILE_NAME "/home/infinity/Code/ros/dataset.bag"

int main(int argc, char **argv) {
    ros::init(argc, argv, "publish_bag");
    ros::start();

    rosbag::Bag bag;
    bag.open(BAG_FILE_NAME, rosbag::bagmode::Read);

    rosbag::View view(bag);

    ros::NodeHandle node_handle;

    /* laser_geometry::LaserProjection projector; */

    std::map<std::string, ros::Publisher> topic_publishers;

    // build topic->publisher map
    for (const rosbag::ConnectionInfo *c : view.getConnections()) {
        ros::AdvertiseOptions options{c->topic, 1, c->md5sum, c->datatype, c->msg_def};
        topic_publishers[c->topic] = node_handle.advertise(options);
    }

    // add /cloud topic
    /* topic_publishers["/cloud"] = node_handle.advertise<sensor_msgs::PointCloud2>("/cloud", 1); */

    ros::Time start_time = ros::Time::now();
    ros::Time bag_start_time = view.getBeginTime();

    ros::Duration delta_time = start_time - bag_start_time;

    for (const rosbag::MessageInstance &m : view) {
        if (!ros::ok()) break;

        ros::Duration t_sequence = m.getTime() - bag_start_time;

        ros::Time::sleepUntil(start_time + t_sequence);

        auto &pub = topic_publishers[m.getTopic()];

        /* printf("%s\n", m.getTopic().c_str()); */

        if (m.isType<sensor_msgs::LaserScan>()) {
            auto message = m.instantiate<sensor_msgs::LaserScan>(); 
            message->header.stamp += delta_time;
            pub.publish(message);


            /* sensor_msgs::PointCloud2 cloud; */
            /* projector.projectLaser(*message, cloud); */
            /* auto &cloud_pub = topic_publishers["/cloud"]; */
            /* cloud_pub.publish(cloud); */
        } else if (m.isType<nav_msgs::Odometry>()) {
            auto message = m.instantiate<nav_msgs::Odometry>(); 
            message->header.stamp += delta_time;
            pub.publish(message);
        } else if (m.isType<tf2_msgs::TFMessage>()) {
            auto message = m.instantiate<tf2_msgs::TFMessage>(); 

            for (auto &tf : message->transforms) {
                tf.header.stamp += delta_time;
            }
            if (m.getTopic() == "/tf_static") {
                printf("%s\n", m.getDataType().c_str());
                printf("%f\n", message->transforms[0].transform.rotation.w);
                printf("%s\n", pub.getTopic().c_str());
                print("%s")
                printf("time: %f\n", message->transforms[0].header.stamp.toSec());
                printf("Latched: %d\n", pub.isLatched());
            }
            pub.publish(message);
        } else if (m.isType<sensor_msgs::Imu>()) {
            auto message = m.instantiate<sensor_msgs::Imu>(); 
            message->header.stamp += delta_time;
            pub.publish(message);
        }
    }

    // cleanup
    bag.close();
    ros::shutdown();
}
