#include "map_loader.h"

MapLoader::MapLoader(ros::NodeHandle &nh, ros::NodeHandle &pnh): nh_(nh), pnh_(pnh){
  read_rosparameters_();
  
  map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/map_topic", 1, true);
  initial_pose_sub_ = nh.subscribe("/robot_pose_topic", 1, &MapLoader::robot_pose_callback_, this);
  ndt_pose_sub_ = nh.subscribe("/init_pose_topic", 10, &MapLoader::init_pose_callback_, this);
  
  load_pcd_(pcd_file_path_);
  transform_map_();
  publish_map_();
}

void MapLoader::read_rosparameters_(){
  pnh_.param<std::string>("pcd_path", pcd_file_path_, "");
  pnh_.param<float>("x", tf_x_, 0.0);
  pnh_.param<float>("y", tf_y_, 0.0);
  pnh_.param<float>("z", tf_z_, 0.0);
  pnh_.param<float>("roll", tf_roll_, 0.0);
  pnh_.param<float>("pitch", tf_pitch_, 0.0);
  pnh_.param<float>("yaw", tf_yaw_, 0.0);
}

void MapLoader::load_pcd_(const std::string &path_to_pcd) {
  map_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(path_to_pcd, *map_) == -1) {
    ROS_ERROR_STREAM("Lidar map not found on path " << path_to_pcd);
  }
  ROS_INFO_STREAM("[MAP_LOADER] -- map is loaded! " << map_->size());
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
  
}

void MapLoader::switch_map(double x, double y) {
  double m11_2_x = -3246.64233398; 
  double m11_2_y = 4485.08544922; 
  std::string m11_2_path = "/workspace/bags/m11/m11_2.pcd";
  double dx = x - m11_2_x;
  double dy = y - m11_2_y;

  double distance_to_new_map = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
  ROS_WARN_STREAM("distance to new map: " << distance_to_new_map);

  bool m11_2_map = distance_to_new_map <= 10.0;

  if (m11_2_map && !m11_2_map_used_) {
    load_pcd_(m11_2_path);
    tf_x_ = -3104.7783200000013;
    tf_y_ = 3651.914062;
    tf_z_ = -8.465209999999999;
    transform_map_();
    publish_map_();
    ROS_WARN_STREAM("M11 2 MAP!");
    m11_2_map_used_ = true;
  }
}


void MapLoader::robot_pose_callback_(const nav_msgs::Odometry::ConstPtr &ndt_odom_msg)
{
  switch_map(ndt_odom_msg->pose.pose.position.x, ndt_odom_msg->pose.pose.position.y);
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
