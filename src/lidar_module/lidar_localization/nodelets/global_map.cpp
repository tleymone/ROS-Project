#include <mutex>
#include <memory>
#include <iostream>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>

#include <sensor_msgs/PointCloud2.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <pcl/filters/voxel_grid.h>

namespace lidar_localization {

    using PointT = pcl::PointXYZI;

    class GlobalMapNodelet : public nodelet::Nodelet{
	
	public:

		GlobalMapNodelet(){}
		virtual ~GlobalMapNodelet() {}
		virtual void onInit(){
			nh = getNodeHandle();
			private_nh = getPrivateNodeHandle();
			globalmap_pub = nh.advertise<sensor_msgs::PointCloud2>("global_map", 5, true);
			initialize_params();
			publish();
		}

	private:

		void initialize_params(){
			/* read global map from pcd file */
			std::string globalmap_pcd = private_nh.param<std::string>("globalmap_pcd","");
			globalmap.reset(new pcl::PointCloud<PointT>());
			pcl::io::loadPCDFile(globalmap_pcd, *globalmap);
			globalmap->header.frame_id = "map";

			/* downsample */
			double downsample_resolution = private_nh.param<double>("downsample_resolution", 0.1);
			boost::shared_ptr<pcl::VoxelGrid<PointT>> voxelgrid(new pcl::VoxelGrid<PointT>());
			voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
			voxelgrid->setInputCloud(globalmap);

			pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
			voxelgrid->filter(*filtered);

			// pcl::PointCloud<PointT>::Ptr testfiltered(new pcl::PointCloud<PointT>());
			// testfiltered->header = filtered->header;
			// testfiltered->header.frame_id = "map";
			// testfiltered->resize(filtered->size());
			// for(size_t i = 0; i < filtered->size(); i++) {
			// 	if(filtered->points[i].x < 50 &&  filtered->points[i].x > -50){
			// 		testfiltered->points[i].x = filtered->points[i].x;
			// 		testfiltered->points[i].y = filtered->points[i].y;
			// 		testfiltered->points[i].z = filtered->points[i].z;
			// 		testfiltered->points[i].intensity = filtered->points[i].intensity;
			// 	}
			// }

			globalmap = filtered;
		}

		void publish(){
			globalmap_pub.publish(globalmap);
		};

	private:
		ros::NodeHandle nh;
		ros::NodeHandle private_nh;

		ros::Publisher globalmap_pub;

		ros::WallTimer globalmap_pub_timer;
		pcl::PointCloud<PointT>::Ptr globalmap;

	};
}

PLUGINLIB_EXPORT_CLASS(lidar_localization::GlobalMapNodelet, nodelet::Nodelet)

