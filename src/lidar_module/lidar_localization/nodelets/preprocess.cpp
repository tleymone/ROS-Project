#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <ros/time.h>
#include <string>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>

namespace lidar_localization {

    // Pandora pointcloud type
    struct EIGEN_ALIGN16 PointPandora{
    PCL_ADD_POINT4D
    uint8_t intensity;
    double timestamp;
    uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    using PointT = pcl::PointXYZI;
    using PointCloudTPandora = pcl::PointCloud<PointPandora>;
    
    class PreprocessNodelet : public nodelet::Nodelet{
	
	public:
		PreprocessNodelet(){}
		virtual ~PreprocessNodelet() {}
		virtual void onInit() {
			private_nh = getPrivateNodeHandle();  
			sub_points = getNodeHandle().subscribe("pandora/points", 10, &PreprocessNodelet::callback, this);
			pub_points = getNodeHandle().advertise<pcl::PointCloud<PointT>>("velodyne_points", 10);
			initialize_params();
		}

	private:
		void initialize_params() {

			/* initialization of downsample filter */
			std::string downsample_method = private_nh.param<std::string>("downsample_method", "VOXELGRID");
			double downsample_resolution = private_nh.param<double>("downsample_resolution", 0.1);

			if(downsample_method == "VOXELGRID") {
			std::cout << "downsample: VOXELGRID " << downsample_resolution << std::endl;
			auto voxelgrid = new pcl::VoxelGrid<PointT>();
			voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
			downsample_filter.reset(voxelgrid);
			} 
			
			else if(downsample_method == "APPROX_VOXELGRID") {
			std::cout << "downsample: APPROX_VOXELGRID " << downsample_resolution << std::endl;
			pcl::ApproximateVoxelGrid<PointT>::Ptr approx_voxelgrid(new pcl::ApproximateVoxelGrid<PointT>());
			approx_voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
			downsample_filter = approx_voxelgrid;
			} 
			
			else {
			if(downsample_method != "NONE") {
				std::cerr << "warning: unknown downsampling type (" << downsample_method << ")" << std::endl;
				std::cerr << "       : use passthrough filter" << std::endl;
			}
			std::cout << "downsample: NONE" << std::endl;
			}

			/* Initialization of outlier filter */
			std::string outlier_removal_method = private_nh.param<std::string>("outlier_removal_method", "STATISTICAL");
			if(outlier_removal_method == "STATISTICAL") {
			int mean_k = private_nh.param<int>("statistical_mean_k", 20);
			double stddev_mul_thresh = private_nh.param<double>("statistical_stddev", 1.0);
			std::cout << "outlier_removal: STATISTICAL " << mean_k << " - " << stddev_mul_thresh << std::endl;

			pcl::StatisticalOutlierRemoval<PointT>::Ptr sor(new pcl::StatisticalOutlierRemoval<PointT>());
			sor->setMeanK(mean_k);
			sor->setStddevMulThresh(stddev_mul_thresh);
			outlier_removal_filter = sor;
			} 
			
			else if(outlier_removal_method == "RADIUS") {
			double radius = private_nh.param<double>("radius_radius", 0.8);
			int min_neighbors = private_nh.param<int>("radius_min_neighbors", 2);
			std::cout << "outlier_removal: RADIUS " << radius << " - " << min_neighbors << std::endl;

			pcl::RadiusOutlierRemoval<PointT>::Ptr rad(new pcl::RadiusOutlierRemoval<PointT>());
			rad->setRadiusSearch(radius);
			rad->setMinNeighborsInRadius(min_neighbors);
			outlier_removal_filter = rad;
			} 
			
			else {
			std::cout << "outlier_removal: NONE" << std::endl;
			}

			/* Initialization of distance and height filter */
			use_distance_filter = private_nh.param<bool>("use_distance_filter", true);
			distance_near_thresh = private_nh.param<double>("distance_near_thresh", 1.0);
			distance_far_thresh = private_nh.param<double>("distance_far_thresh", 50.0);
			height_max = private_nh.param<double>("height_max", 10.0);
			height_min = private_nh.param<double>("height_min", -5.0);

			/* Initialization of clock delay */
			clock_delay = private_nh.param<double>("clock_delay", 25199.840000);
			clock_delay = clock_delay * 1e6;
		}

		void callback(PointCloudTPandora::ConstPtr pc)
		{
			/* convert to XYZI type */
			pcl::PointCloud<PointT>::Ptr nonfiltered(new pcl::PointCloud<PointT>());
			nonfiltered->header = pc->header;
			nonfiltered->header.frame_id = "velodyne";
			nonfiltered->header.stamp = pc->header.stamp - clock_delay;
			nonfiltered->resize(pc->size());
			for(size_t i = 0; i < nonfiltered->size(); i++) {
				nonfiltered->points[i].x = pc->points[i].x;
				nonfiltered->points[i].y = pc->points[i].y;
				nonfiltered->points[i].z = pc->points[i].z;
				nonfiltered->points[i].intensity = (float)(pc->points[i].intensity);
			}

			/* filter */
			pcl::PointCloud<PointT>::ConstPtr filtered = distance_filter(nonfiltered); 
			filtered = downsample(filtered);
			filtered = outlier_removal(filtered);

			pub_points.publish(filtered);
		}

		/* distance and height filter */
		pcl::PointCloud<PointT>::ConstPtr distance_filter(const pcl::PointCloud<PointT>::ConstPtr& cloud) const {
		pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());

		filtered->reserve(cloud->size());
		std::copy_if(cloud->begin(), cloud->end(), std::back_inserter(filtered->points), [&](const PointT& p) {
		double d = p.getVector3fMap().norm();
		return d > distance_near_thresh && d < distance_far_thresh && p.z > height_min && p.z < height_max;
		});

		filtered->width = filtered->size();
		filtered->height = 1;
		filtered->is_dense = false;

		filtered->header = cloud->header;

		return filtered;
		}
		
		/* downsample filter */
		pcl::PointCloud<PointT>::ConstPtr downsample(const pcl::PointCloud<PointT>::ConstPtr& cloud) const {
		if(!downsample_filter) {
			return cloud;
		}

		pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
		downsample_filter->setInputCloud(cloud);
		downsample_filter->filter(*filtered);
		filtered->header = cloud->header;

		return filtered;
		}

		/* outlier filter */
		pcl::PointCloud<PointT>::ConstPtr outlier_removal(const pcl::PointCloud<PointT>::ConstPtr& cloud) const {
		if(!outlier_removal_filter) {
		return cloud;
		}

		pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
		outlier_removal_filter->setInputCloud(cloud);
		outlier_removal_filter->filter(*filtered);
		filtered->header = cloud->header;

		return filtered;
		}

	private:
		bool use_distance_filter;
		double height_max;
		double height_min;
		double distance_near_thresh;
		double distance_far_thresh;
		double clock_delay;

		ros::Publisher pub_points;
		ros::Subscriber sub_points;
		ros::NodeHandle private_nh;

		pcl::Filter<PointT>::Ptr downsample_filter;
		pcl::Filter<PointT>::Ptr outlier_removal_filter;
    };
}

POINT_CLOUD_REGISTER_POINT_STRUCT(lidar_localization::PointPandora,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (std::uint8_t, intensity, intensity)
                                  (std::uint16_t, ring, ring)
                                  (double, timestamp, timestamp))

PLUGINLIB_EXPORT_CLASS(lidar_localization::PreprocessNodelet, nodelet::Nodelet)

