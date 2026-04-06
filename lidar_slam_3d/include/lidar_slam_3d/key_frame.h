#ifndef KEY_FRAME_H
#define KEY_FRAME_H

#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "types.h"

namespace lidar_slam_3d
{

class KeyFrame
{
public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<KeyFrame> Ptr;
    KeyFrame() {}
    //KeyFrame(int id, const Eigen::Matrix4f& pose, const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
    KeyFrame(int id, const Eigen::Matrix4f& pose, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)  // TODO @Yimin Zhao
    {
        id_ = id;
        pose_ = pose;
        cloud_ = cloud;
    }
    ~KeyFrame() {}

    void setId(int id) { id_ = id; }
    void setPose(const Eigen::Matrix4f& pose) {
        pose_ = pose; 
        pq_.p = pose.block<3,1>(0,3).template cast<double>(); 
        pq_.q = Eigen::Quaterniond(pose.block<3,3>(0,0).template cast<double>()); 
    }

    void setPosePQ(const Eigen::Vector3d& p, const Eigen::Quaterniond& q){
        pq_.p = p;
        pq_.q = q;
        pose_.block<3,1>(0,3) = p.template cast<float>();
        pose_.block<3,3>(0,0) = q.toRotationMatrix().template cast<float>();
    }
    //void setCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) { cloud_ = cloud; }
    void setCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) { cloud_ = cloud; }  // TODO @Yimin Zhao

    int getId() { return id_; }
    Eigen::Matrix4f getPose() { return pose_; }
    // Retrun PQ
    PosePQ getPosePQ(){return pq_;}
    //pcl::PointCloud<pcl::PointXYZI>::Ptr getCloud() { return cloud_; }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCloud() { return cloud_; } // TODO @Yimin Zhao

private:
    int id_;
    Eigen::Matrix4f pose_;
    // For ceres pose optimization
    PosePQ pq_;
    //pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_; // TODO @Yimin Zhao
};

} // namespace lidar_slam_3d

#endif // KEY_FRAME_H
