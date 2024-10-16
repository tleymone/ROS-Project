#ifndef POSE_ESTIMATOR_HPP
#define POSE_ESTIMATOR_HPP

#include <memory>
#include <boost/optional.hpp>

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/registration.h>

namespace lidar_localization{

class PoseEstimator{
public:
    using PointT = pcl::PointXYZI;

    /**
     * @brief constructor
     * @param registration        registration method
     * @param stamp               timestamp
     * @param pos                 initial position, get from Kalman filter 
     * @param quat                initial orientation, get from Kalman filter
     */
    PoseEstimator(pcl::Registration<PointT, PointT>::Ptr& registration, const ros::Time& stamp, const Eigen::Vector3f& pos, const Eigen::Quaternionf& quat):
    init_stamp(stamp), registration(registration), pos(pos), quat(quat) {}
    ~PoseEstimator(){}

    /**
     * @brief correct
     * @param stamp time stamp
     * @param cloud   input cloud
     * @param init_guess Eigen::Matrix4f
     * @return transformation to the globalmap
     */
    pcl::PointCloud<PointT>::Ptr correct(const ros::Time& stamp, const pcl::PointCloud<PointT>::ConstPtr& cloud, const Eigen::Matrix4f& init_guess){
        pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>());
        registration->setInputSource(cloud);
        registration->align(*aligned, init_guess);

        Eigen::Matrix4f trans = registration->getFinalTransformation();
        pos = trans.block<3, 1>(0, 3);
        Eigen::Quaternionf q(trans.block<3, 3>(0, 0));
        quat = q;
        
        return aligned;
    };
    
    Eigen::Matrix4f matrix() const {
        Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
        m.block<3, 3>(0, 0) = getquat().toRotationMatrix();
        m.block<3, 1>(0, 3) = getpos();
        return m;
    };

    Eigen::Vector3f getpos() const {
        return pos;
    }

    Eigen::Quaternionf getquat() const {
        return quat.normalized();
    }

    private:
    ros::Time init_stamp;
    pcl::Registration<PointT, PointT>::Ptr registration;
    Eigen::Vector3f pos;
    Eigen::Quaternionf quat;
};

}

#endif