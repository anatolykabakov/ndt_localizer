#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <pcl_ros/transforms.h>
#include <unordered_map>

class MapLoader{
public:
    MapLoader(ros::NodeHandle &nh, ros::NodeHandle &pnh);

private:
    std::vector<std::string> file_list_;
    ros::Publisher map_pub_;
    ros::Subscriber ndt_pose_sub_;
    ros::Subscriber initial_pose_sub_;
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    geometry_msgs::Pose curr_pose_;
    geometry_msgs::Pose pre_pose_;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_;

    std::string map_topic_;
    std::string init_pose_topic_;
    std::string robot_pose_topic_;
    std::string pcd_file_path_;

    float submap_size_xy_;
    float submap_size_z_;
    float traversal_dist_=0.;
    float map_switch_thres_;
    float tf_x_;
    float tf_y_;
    float tf_z_;
    float tf_roll_;
    float tf_pitch_;
    float tf_yaw_; 

    bool m11_2_map_used_ = false;

    std::unordered_map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> maps_;
    std::unordered_map<std::string, bool> maps_used_;
private:
    void read_rosparameters_();
    void load_pcd_(const std::string &path_to_pcd);
    void transform_map_();
    void publish_map_();

    void switch_map(double x, double y);

    void load_maps_(const std::string &path);

    void robot_pose_callback_(const nav_msgs::Odometry::ConstPtr &ndt_odom_msg);
    void init_pose_callback_(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &initial_pose_msg_ptr);
};

