#include "map_loader.h"
#include <experimental/filesystem>
#include <sstream>
#include <string>

namespace fs = std::experimental::filesystem;

MapLoader::MapLoader(ros::NodeHandle &nh, ros::NodeHandle &pnh): nh_(nh), pnh_(pnh){
  read_rosparameters_();
  load_maps_(pcd_file_path_);
  
  map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/map_topic", 1, true);
  initial_pose_sub_ = nh.subscribe("/robot_pose_topic", 1, &MapLoader::robot_pose_callback_, this);
  ndt_pose_sub_ = nh.subscribe("/init_pose_topic", 10, &MapLoader::init_pose_callback_, this);
  
}

void MapLoader::load_maps_(const std::string &path) {
  for (const auto path : fs::directory_iterator(path)) {
    if (path.path().extension() != ".pcd") {
      continue;
    }
    std::cout << path << " loaded"  << std::endl;
    maps_used_.insert(std::make_pair(path.path().string(), false));
  }
}

void MapLoader::read_rosparameters_(){
  pnh_.param<std::string>("pcd_path", pcd_file_path_, "");
  pnh_.param<float>("submap_size_xy", submap_size_xy_, 100);
  pnh_.param<float>("submap_size_z", submap_size_z_, 50);
  pnh_.param<float>("map_switch_thres", map_switch_thres_, 10);

}

void MapLoader::load_pcd_(const std::string &path_to_pcd) {
  map_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(path_to_pcd, *map_) == -1) {
    ROS_ERROR_STREAM("Lidar map not found on path " << path_to_pcd);
  }
}

void MapLoader::transform_map_() {

  Eigen::Translation3f tl_m2w(tf_x_, tf_y_, tf_z_);                 // tl: translation
  Eigen::AngleAxisf rot_x_m2w(tf_roll_, Eigen::Vector3f::UnitX());  // rot: rotation
  Eigen::AngleAxisf rot_y_m2w(tf_pitch_, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rot_z_m2w(tf_yaw_, Eigen::Vector3f::UnitZ());
  Eigen::Matrix4f tf_m2w = (tl_m2w * rot_z_m2w * rot_y_m2w * rot_x_m2w).matrix();

  pcl::transformPointCloud(*map_, *map_, tf_m2w);
}

void MapLoader::publish_map_() {
  sensor_msgs::PointCloud2::Ptr map_msg(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*map_, *map_msg);
  map_msg->header.frame_id = "map";
  map_pub_.publish(*map_msg);
  ROS_INFO_STREAM("[MAP_LOADER] -- new map published!");
}

void MapLoader::init_pose_callback_(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &initial_pose_msg_ptr)
{
  switch_map(initial_pose_msg_ptr->pose.pose.position.x, initial_pose_msg_ptr->pose.pose.position.y);
  curr_pose_ = initial_pose_msg_ptr->pose.pose;
  
  pcl::CropBox<pcl::PointXYZ> box_filter;
  box_filter.setMin(Eigen::Vector4f(curr_pose_.position.x-submap_size_xy_, curr_pose_.position.y-submap_size_xy_, curr_pose_.position.z-submap_size_z_, 1.0));
  box_filter.setMax(Eigen::Vector4f(curr_pose_.position.x+submap_size_xy_, curr_pose_.position.y+submap_size_xy_, curr_pose_.position.z+submap_size_z_, 1.0));
  pcl::PointCloud<pcl::PointXYZ>::Ptr submap_ptr(new pcl::PointCloud<pcl::PointXYZ>());
  box_filter.setInputCloud(map_);
  box_filter.filter(*submap_ptr);
  
  sensor_msgs::PointCloud2::Ptr map_msg(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*submap_ptr, *map_msg);
  map_msg->header.frame_id = "map";
  map_pub_.publish(*map_msg);

  pre_pose_ = curr_pose_;
}

void MapLoader::switch_map(double x, double y) {
  
  for (const auto &[map_path, used_flag] : maps_used_) {
    const auto map_name = fs::path(map_path).stem();
    std::stringstream tmp(map_name);
    std::vector<std::string> items;
    std::string element;

    while(std::getline(tmp, element, ',')) {
      items.push_back(element);
    }

    double map_x = std::stod(items[0]);
    double map_y = std::stod(items[1]);
    double map_z = std::stod(items[2]);

    double dx = x - map_x;
    double dy = y - map_y;
    double distance_to_new_map = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
    if (distance_to_new_map <= 20.0 && !maps_used_.at(map_path)) {
      load_pcd_(map_path);
      
      tf_x_ = map_x;
      tf_y_ = map_y;
      tf_z_ = map_z;
      transform_map_();

      for (auto &[key, value] : maps_used_) {
        maps_used_[key] = false;
      }
      maps_used_[map_path] = true;
      break;
    }
  }
}


void MapLoader::robot_pose_callback_(const nav_msgs::Odometry::ConstPtr &ndt_odom_msg)
{
  switch_map(ndt_odom_msg->pose.pose.position.x, ndt_odom_msg->pose.pose.position.y);
  curr_pose_ = ndt_odom_msg->pose.pose;

  traversal_dist_ += sqrt(pow(curr_pose_.position.x-pre_pose_.position.x,2) + pow(curr_pose_.position.y-pre_pose_.position.y,2));

  if(traversal_dist_>= map_switch_thres_) {
    pcl::CropBox<pcl::PointXYZ> box_filter;
    box_filter.setMin(Eigen::Vector4f(curr_pose_.position.x-submap_size_xy_, curr_pose_.position.y-submap_size_xy_, curr_pose_.position.z-submap_size_z_, 1.0));
    box_filter.setMax(Eigen::Vector4f(curr_pose_.position.x+submap_size_xy_, curr_pose_.position.y+submap_size_xy_, curr_pose_.position.z+submap_size_z_, 1.0));
    pcl::PointCloud<pcl::PointXYZ>::Ptr submap_ptr(new pcl::PointCloud<pcl::PointXYZ>());
    box_filter.setInputCloud(map_);
    box_filter.filter(*submap_ptr);
    
    sensor_msgs::PointCloud2::Ptr map_msg(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*submap_ptr, *map_msg);
    map_msg->header.frame_id = "map";
    map_pub_.publish(*map_msg);

    ROS_INFO("new submap is published!");
	
    traversal_dist_ = 0;
  }
  pre_pose_ = curr_pose_;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "map_loader");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  MapLoader map_loader(nh, pnh);
  ros::spin();

  return 0;
}
