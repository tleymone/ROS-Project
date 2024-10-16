#include <mutex>
#include <memory>
#include <iostream>
#include <math.h>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <eigen_conversions/eigen_msg.h>

#include <std_srvs/Empty.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <pcl/filters/voxel_grid.h>

#include <pclomp/ndt_omp.h>

#include "lidar_localization/QueryGlobalLocalization.h"
#include "lidar_localization/SetGlobalMap.h"
#include "pose_estimator.hpp"
#include "delta_estimate.hpp"

#include "lidar_localization/QueryGlobalLocalization.h"
#include "lidar_localization/SetGlobalMap.h"

namespace lidar_localization {

class LidarLocalizationNodelet : public nodelet::Nodelet {
public:
  using PointT = pcl::PointXYZI;

  LidarLocalizationNodelet() : tf_buffer(), tf_listener(tf_buffer) {}
  virtual ~LidarLocalizationNodelet() {}

  void onInit() override {
    nh = getNodeHandle();
    mt_nh = getMTNodeHandle();
    private_nh = getPrivateNodeHandle();

    initialize_params();

    points_sub = mt_nh.subscribe("/velodyne_points", 5, &LidarLocalizationNodelet::points_callback, this);
    globalmap_sub = nh.subscribe("/global_map", 1, &LidarLocalizationNodelet::globalmap_callback, this);
    initialpose_sub = nh.subscribe("/initialpose", 8, &LidarLocalizationNodelet::initialpose_callback, this);
    //pose_sub = nh.subscribe("/span/enuposCovs", 5, &LidarLocalizationNodelet::rtkpose_callback, this);
    kalman_sub = nh.subscribe("/kalman", 5, &LidarLocalizationNodelet::kalmanpose_callback, this);

    pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/lidar_pose", 5, false);
    aligned_pub = nh.advertise<sensor_msgs::PointCloud2>("/aligned_points", 5, false);

    // initialize global localization
    NODELET_INFO_STREAM("wait for global localization services");
    ros::service::waitForService("/lidar_global_localization/set_global_map");
    ros::service::waitForService("/lidar_global_localization/query");

    set_global_map_service = nh.serviceClient<lidar_localization::SetGlobalMap>("/lidar_global_localization/set_global_map");
    query_global_localization_service = nh.serviceClient<lidar_localization::QueryGlobalLocalization>("/lidar_global_localization/query");

    relocalize_server = nh.advertiseService("/relocalize", &LidarLocalizationNodelet::relocalize, this);

    initialize_tf_enu2map();
    initialize_tf_map_pc2map();
  }

private:
  pcl::Registration<PointT, PointT>::Ptr create_registration() const {
    std::string reg_method = private_nh.param<std::string>("reg_method", "NDT_OMP");
    std::string ndt_neighbor_search_method = private_nh.param<std::string>("ndt_neighbor_search_method", "DIRECT7");
    double ndt_neighbor_search_radius = private_nh.param<double>("ndt_neighbor_search_radius", 2.0);
    double ndt_resolution = private_nh.param<double>("ndt_resolution", 1.0);

    if(reg_method == "NDT_OMP") {
      NODELET_INFO("NDT_OMP is selected");
      pclomp::NormalDistributionsTransform<PointT, PointT>::Ptr ndt(new pclomp::NormalDistributionsTransform<PointT, PointT>());
      ndt->setTransformationEpsilon(0.01);
      ndt->setResolution(ndt_resolution);
      if (ndt_neighbor_search_method == "DIRECT1") {
        NODELET_INFO("search_method DIRECT1 is selected");
        ndt->setNeighborhoodSearchMethod(pclomp::DIRECT1);
      } else if (ndt_neighbor_search_method == "DIRECT7") {
        NODELET_INFO("search_method DIRECT7 is selected");
        ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
      } else {
        if (ndt_neighbor_search_method == "KDTREE") {
          NODELET_INFO("search_method KDTREE is selected");
        } else {
          NODELET_WARN("invalid search method was given");
          NODELET_WARN("default method is selected (KDTREE)");
        }
        ndt->setNeighborhoodSearchMethod(pclomp::KDTREE);
      }
      return ndt;
    } 

    NODELET_ERROR_STREAM("initialization registor is failed !!!");
    return nullptr;
  }

  void initialize_params() {

    NODELET_INFO("create registration method for localization");
    registration = create_registration();

    // global localization
    NODELET_INFO("create registration method for callback during relocalization");
    relocalizing = false;
    delta_estimater.reset(new DeltaEstimater(create_registration()));

    // initialize pose estimator
    if(private_nh.param<bool>("specify_init_pose", true)) {
      NODELET_INFO("initialize pose estimator with specified parameters!!");
      pose_estimator.reset(new PoseEstimator(registration,
        ros::Time::now(),
        Eigen::Vector3f(private_nh.param<double>("init_pos_x", 0.0), private_nh.param<double>("init_pos_y", 0.0), private_nh.param<double>("init_pos_z", 0.0)),
        Eigen::Quaternionf(private_nh.param<double>("init_ori_w", 1.0), private_nh.param<double>("init_ori_x", 0.0), private_nh.param<double>("init_ori_y", 0.0), private_nh.param<double>("init_ori_z", 0.0))));
    }
  }

  void initialize_tf_enu2map() {
    
    /* lidar_enu to lidar_map */
    double tx = private_nh.param<double>("init_enu_pos_x", 0.0);
    double ty = private_nh.param<double>("init_enu_pos_y", 0.0);
    double tz = private_nh.param<double>("init_enu_pos_z", 0.0);

    double qx = private_nh.param<double>("init_enu_ori_x", 0.0);
    double qy = private_nh.param<double>("init_enu_ori_y", 0.0);
    double qz = private_nh.param<double>("init_enu_ori_z", 0.0);
    double qw = private_nh.param<double>("init_enu_ori_w", 0.0);

    theta0 = atan2(2.0*(qx*qy + qz*qw), 1.0 - 2.0*(qz*qz + qy*qy)) - 0.06;

    tf::StampedTransform lidar_enu;
    tf::Quaternion q(qx, qy, qz, qw);
    tf::Quaternion q1;
    q1.setRPY(0, 0, theta0);
    tf::Vector3 t(tx + cos(theta0) * 1.157, ty + sin(theta0) * 1.157, tz + 1.61);
    
    tf::Quaternion q_span2lidar(0, 0, 0.707073, 0.707073);
    q = q_span2lidar * q;
    q.normalize();
    lidar_enu.setOrigin(t);
    lidar_enu.setRotation(q);
    lidar_enu.frame_id_ = "enu";
    lidar_enu.child_frame_id_ = "map_pc";
    lidar_enu.stamp_ = ros::Time::now();

    geometry_msgs::TransformStamped lidar_odom;
    tf::transformStampedTFToMsg(lidar_enu, lidar_odom);
    static_tf_broadcaster.sendTransform(lidar_odom);
  }

  void initialize_tf_map_pc2map() {
    geometry_msgs::TransformStamped map_pc2map;
    map_pc2map.transform.translation.x = 0;
    map_pc2map.transform.translation.y = 0;
    map_pc2map.transform.translation.z = 0;
    map_pc2map.transform.rotation.x = 0;
    map_pc2map.transform.rotation.y = 0;
    map_pc2map.transform.rotation.z = 0;    
    map_pc2map.transform.rotation.w = 1;
    map_pc2map.child_frame_id = "map";
    map_pc2map.header.frame_id = "map_pc";
    static_tf_broadcaster.sendTransform(map_pc2map);
  }

private:

  /**
   * @brief callback for point cloud data
   * @param points_msg
   */
  void points_callback(const sensor_msgs::PointCloud2ConstPtr& points_msg) {
    //
    std::lock_guard<std::mutex> estimator_lock(pose_estimator_mutex);
    if(!pose_estimator) {
      NODELET_ERROR("waiting for initial pose input!!");
      return;
    }

    if(!globalmap) {
      NODELET_ERROR("globalmap has not been received!!");
      return;
    }
    // msg to pcl
    const auto& stamp = points_msg->header.stamp;

    if (stamp.toSec() - last_stamp.toSec() < 0.35){
      return;
    }

    last_stamp = stamp;

    pcl::PointCloud<PointT>::Ptr pcl_cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*points_msg, *pcl_cloud);

    if(pcl_cloud->empty()) {
      NODELET_ERROR("cloud is empty!!");
      return;
    }

    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    *cloud = *pcl_cloud;

    last_scan = cloud;

    if(relocalizing) {
      delta_estimater->add_frame(cloud);
    }
    
    Eigen::Matrix4f init_guess;
    geometry_msgs::TransformStamped init;
    
    if(!tf_buffer.canTransform("map_pc", "velodyne_kalman", stamp){
      return;
    };

    init = tf_buffer.lookupTransform("map_pc", "velodyne_kalman", stamp);
    init_guess.block<3,1>(0,3) = Eigen::Vector3f(init.transform.translation.x, init.transform.translation.y, 0);
    init_guess.block<3,3>(0,0) = Eigen::Quaternionf(init.transform.rotation.w, init.transform.rotation.x, init.transform.rotation.y, init.transform.rotation.z).toRotationMatrix();

    auto aligned = pose_estimator->correct(stamp, cloud, init_guess);
    aligned->header.frame_id = "map";
    aligned->header.stamp = cloud->header.stamp;
    aligned_pub.publish(aligned);

    publish_odometry(points_msg->header.stamp, pose_estimator->matrix());

  }

  /**
   * @brief publish odometry
   * @param stamp  timestamp
   * @param pose   odometry pose to be published
   */
  void publish_odometry(const ros::Time& stamp, const Eigen::Matrix4f& pose) {

    // broadcast the transform over tf
    geometry_msgs::TransformStamped odom_trans = tf2::eigenToTransform(Eigen::Isometry3d(pose.cast<double>()));
    odom_trans.transform.translation.z = 0;
    odom_trans.header.stamp = stamp;
    odom_trans.header.frame_id = "map_pc";
    odom_trans.child_frame_id = "velodyne";
    tf_broadcaster.sendTransform(odom_trans);

    // send pose message
    geometry_msgs::PoseWithCovarianceStamped lidar_pose;
    double theta = atan2(2.0*(odom_trans.transform.rotation.z*odom_trans.transform.rotation.w + odom_trans.transform.rotation.x*odom_trans.transform.rotation.y),
                              1.0 - 2.0*(odom_trans.transform.rotation.y*odom_trans.transform.rotation.y + odom_trans.transform.rotation.z*odom_trans.transform.rotation.z));
    tf::Quaternion q;
    double angle;
    angle = theta + theta0;
    if(theta + theta0 > 3.1415926){
      angle -= 3.1415926*2;
    }
    if(theta + theta0 < -3.1415926){
      angle += 3.1415926*2; 
    }
    q.setRPY(0, 0, angle);

    lidar_pose.pose.pose.orientation.x = q.x();
    lidar_pose.pose.pose.orientation.y = q.y();
    lidar_pose.pose.pose.orientation.z = q.z();
    lidar_pose.pose.pose.orientation.w = q.w();

    geometry_msgs::TransformStamped enu2map = tf_buffer.lookupTransform("enu","map_pc",ros::Time::now());
    double theta_pos = atan2(odom_trans.transform.translation.x, - odom_trans.transform.translation.y);
    lidar_pose.pose.pose.position.x = enu2map.transform.translation.x + cos(theta_pos+theta0)*sqrt(pow(odom_trans.transform.translation.y,2)+pow(odom_trans.transform.translation.x,2));
    lidar_pose.pose.pose.position.y = enu2map.transform.translation.y + sin(theta_pos+theta0)*sqrt(pow(odom_trans.transform.translation.y,2)+pow(odom_trans.transform.translation.x,2));
    lidar_pose.pose.pose.position.x -= 1.157*cos(theta+theta0);
    lidar_pose.pose.pose.position.y -= 1.157*sin(theta+theta0);

    lidar_pose.header.stamp = stamp;
    lidar_pose.header.frame_id = "map";

    pose_pub.publish(lidar_pose);
  }

  /**
   * @brief update the last pose get from Kalman filter, !!! for debug ...... !!!
   * @param fusionpose_msg 
   */
  void rtkpose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& fusionpose_msg){
    
    // span_enu to lidar_enu
    geometry_msgs::PoseStamped lidar_enu;
    lidar_enu.header = fusionpose_msg->header;
    lidar_enu.pose = fusionpose_msg->pose.pose;
    
    double theta = atan2(2.0*(lidar_enu.pose.orientation.z*lidar_enu.pose.orientation.w + lidar_enu.pose.orientation.x*lidar_enu.pose.orientation.y),
                              1.0 - 2.0*(lidar_enu.pose.orientation.y*lidar_enu.pose.orientation.y + lidar_enu.pose.orientation.z*lidar_enu.pose.orientation.z));
    lidar_enu.pose.position.x += cos(theta) * 1.157;
    lidar_enu.pose.position.y += sin(theta) * 1.157;
    // lidar_enu.pose.position.z -= 1.61;
    tf::Quaternion q(lidar_enu.pose.orientation.x, lidar_enu.pose.orientation.y, lidar_enu.pose.orientation.z, lidar_enu.pose.orientation.w);
    tf::Quaternion q_span2lidar;
    q_span2lidar.setRPY(0, 0, 1.5707);
    q = q_span2lidar*q;
    q.normalize();
    lidar_enu.pose.orientation.x = q.x();
    lidar_enu.pose.orientation.y = q.y();
    lidar_enu.pose.orientation.z = q.z();
    lidar_enu.pose.orientation.w = q.w();

    // lidar_enu to lidar_map
    geometry_msgs::TransformStamped enu2map = tf_buffer.lookupTransform("world","map",ros::Time::now());
    geometry_msgs::PoseStamped lidar_map;
    lidar_map = tf_buffer.transform(lidar_enu, lidar_map, "map", ros::Duration(0.0));

    // lidar_map to lidar_lidar
    tf::StampedTransform lidar_trans;
    lidar_trans.setOrigin(tf::Vector3(lidar_map.pose.position.x, lidar_map.pose.position.y, 0));
    lidar_trans.setRotation(tf::Quaternion(lidar_map.pose.orientation.x, lidar_map.pose.orientation.y,lidar_map.pose.orientation.z, lidar_map.pose.orientation.w));
    lidar_trans.frame_id_ = "map";
    lidar_trans.child_frame_id_ = "velodyne_rtk";
    lidar_trans.stamp_ = fusionpose_msg->header.stamp;
    geometry_msgs::TransformStamped lidar_odom;
    tf::transformStampedTFToMsg(lidar_trans, lidar_odom);
    // tf_broadcaster.sendTransform(lidar_odom);
  }

  /**
   * @brief update the last pose get from Kalman filter
   * @param kalmanpose_msg 
   */
  void kalmanpose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& kalmanpose_msg){
    
    // transform span_enu to lidar_enu
    geometry_msgs::PoseStamped lidar_enu;
    lidar_enu.pose.position.x = kalmanpose_msg->pose.pose.position.x;
    lidar_enu.pose.position.y = kalmanpose_msg->pose.pose.position.y;

    double theta = atan2(2.0*(kalmanpose_msg->pose.pose.orientation.z*kalmanpose_msg->pose.pose.orientation.w + kalmanpose_msg->pose.pose.orientation.x*kalmanpose_msg->pose.pose.orientation.y),
                              1.0 - 2.0*(kalmanpose_msg->pose.pose.orientation.y*kalmanpose_msg->pose.pose.orientation.y + kalmanpose_msg->pose.pose.orientation.z*kalmanpose_msg->pose.pose.orientation.z));
    lidar_enu.pose.position.x += cos(theta) * 1.157;
    lidar_enu.pose.position.y += sin(theta) * 1.157;

    // lidar_enu to lidar_map
    geometry_msgs::TransformStamped enu2map = tf_buffer.lookupTransform("enu","map_pc",ros::Time::now());
    geometry_msgs::PoseStamped lidar_map;
    tf::Quaternion q_enu2map;
    q_enu2map.setRPY(0,0,theta-theta0);
    lidar_map.pose.position.x = cos(theta0)*(lidar_enu.pose.position.y - enu2map.transform.translation.y) - sin(theta0)*(lidar_enu.pose.position.x - enu2map.transform.translation.x);
    lidar_map.pose.position.y = -sin(theta0)*(lidar_enu.pose.position.y - enu2map.transform.translation.y) - cos(theta0)*(lidar_enu.pose.position.x - enu2map.transform.translation.x);
    lidar_map.pose.position.z = 0;
    lidar_map.pose.orientation.x = q_enu2map.x();
    lidar_map.pose.orientation.y = q_enu2map.y();
    lidar_map.pose.orientation.z = q_enu2map.z();
    lidar_map.pose.orientation.w = q_enu2map.w();
    
    // lidar_map to lidar_lidar
    tf::StampedTransform lidar_trans;
    lidar_trans.setOrigin(tf::Vector3(lidar_map.pose.position.x, lidar_map.pose.position.y, lidar_map.pose.position.z));
    lidar_trans.setRotation(tf::Quaternion(lidar_map.pose.orientation.x, lidar_map.pose.orientation.y,lidar_map.pose.orientation.z, lidar_map.pose.orientation.w));
    lidar_trans.stamp_ = kalmanpose_msg->header.stamp;
    lidar_trans.frame_id_ = "map_pc";
    lidar_trans.child_frame_id_ = "velodyne_kalman";
    geometry_msgs::TransformStamped lidar_odom;
    tf::transformStampedTFToMsg(lidar_trans, lidar_odom);
    tf_broadcaster.sendTransform(lidar_odom);
  }

  /**
   * @brief callback for globalmap input
   * @param points_msg
   */
  void globalmap_callback(const sensor_msgs::PointCloud2ConstPtr& points_msg) {
    NODELET_INFO("globalmap received!");
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*points_msg, *cloud);
    globalmap = cloud;

    registration->setInputTarget(globalmap);

    NODELET_INFO("set globalmap for global localization!");
    lidar_localization::SetGlobalMap srv;
    pcl::toROSMsg(*globalmap, srv.request.global_map);

    if(!set_global_map_service.call(srv)) {
      NODELET_INFO("failed to set global map");
    } else {
      NODELET_INFO("done");
    }
  }

  /**
   * @brief perform global localization to relocalize the sensor position
   * @param
   */
  bool relocalize(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res) {
    if(last_scan == nullptr) {
      NODELET_INFO_STREAM("no scan has been received");
      return false;
    }

    relocalizing = true;
    delta_estimater->reset();
    pcl::PointCloud<PointT>::ConstPtr scan = last_scan;

    lidar_localization::QueryGlobalLocalization srv;
    pcl::toROSMsg(*scan, srv.request.cloud);
    srv.request.max_num_candidates = 1;

    if(!query_global_localization_service.call(srv) || srv.response.poses.empty()) {
      relocalizing = false;
      NODELET_INFO_STREAM("global localization failed");
      return false;
    }

    const auto& result = srv.response.poses[0];

    NODELET_INFO_STREAM("--- Global localization result ---");
    NODELET_INFO_STREAM("Trans :" << result.position.x << " " << result.position.y << " " << result.position.z);
    NODELET_INFO_STREAM("Quat  :" << result.orientation.x << " " << result.orientation.y << " " << result.orientation.z << " " << result.orientation.w);
    NODELET_INFO_STREAM("Error :" << srv.response.errors[0]);
    NODELET_INFO_STREAM("Inlier:" << srv.response.inlier_fractions[0]);

    Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
    pose.linear() = Eigen::Quaternionf(result.orientation.w, result.orientation.x, result.orientation.y, result.orientation.z).toRotationMatrix();
    pose.translation() = Eigen::Vector3f(result.position.x, result.position.y, result.position.z);
    pose = pose * delta_estimater->estimated_delta();

    std::lock_guard<std::mutex> lock(pose_estimator_mutex);
    pose_estimator.reset(new PoseEstimator(
      registration,
      ros::Time::now(),
      pose.translation(),
      Eigen::Quaternionf(pose.linear())));
    relocalizing = false;

    return true;
  }

  /**
   * @brief callback for initial pose input ("2D Pose Estimate" on rviz)
   * @param pose_msg
   */
  void initialpose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose_msg) {
    NODELET_INFO("initial pose received!!");
    std::lock_guard<std::mutex> lock(pose_estimator_mutex);
    const auto& p = pose_msg->pose.pose.position;
    const auto& q = pose_msg->pose.pose.orientation;
    pose_estimator.reset(
          new PoseEstimator(
            registration,
            ros::Time::now(),
            Eigen::Vector3f(p.x, p.y, p.z),
            Eigen::Quaternionf(q.w, q.x, q.y, q.z))
    );
  }

private:
  // ROS
  ros::NodeHandle nh;
  ros::NodeHandle mt_nh;
  ros::NodeHandle private_nh;

  ros::Subscriber points_sub;
  ros::Subscriber globalmap_sub;
  ros::Subscriber initialpose_sub;
  ros::Subscriber pose_sub;
  ros::Subscriber kalman_sub;

  ros::Publisher pose_pub;
  ros::Publisher aligned_pub;
  ros::Publisher status_pub;

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;
  tf2_ros::TransformBroadcaster tf_broadcaster;
  tf2_ros::StaticTransformBroadcaster static_tf_broadcaster;

  // globalmap and registration method
  pcl::PointCloud<PointT>::Ptr globalmap;
  pcl::Registration<PointT, PointT>::Ptr registration;

  // pose estimator
  std::mutex pose_estimator_mutex;
  std::unique_ptr<PoseEstimator> pose_estimator;

  // global localization
  std::atomic_bool relocalizing;
  std::unique_ptr<DeltaEstimater> delta_estimater;

  pcl::PointCloud<PointT>::ConstPtr last_scan;
  ros::ServiceServer initialize_server;
  ros::ServiceServer relocalize_server;
  ros::ServiceClient set_global_map_service;
  ros::ServiceClient query_global_localization_service;

  double theta0;
  ros::Time last_stamp;
};

}

PLUGINLIB_EXPORT_CLASS(lidar_localization::LidarLocalizationNodelet, nodelet::Nodelet)
