#ifndef MAP_BUILDER_H
#define MAP_BUILDER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/ndt.h>
#include <ceres/ceres.h>
#include <chrono>
#include <mutex>
#include <utility>
#include "math_func.h"
#include "key_frame.h"
#include "types.h"
#include <cmath>

#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>

typedef Eigen::Matrix<float, 6, 1> Vector6f;

namespace lidar_slam_3d
{

class MapBuilder
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    MapBuilder();
    ~MapBuilder() {}

    //void addPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& point_cloud);
    //void addPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& point_cloud, const Eigen::Matrix4f& guess_pose); // TODO @Yimin Zhao
    void addPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& point_cloud, double angley); // TODO @Yimin Zhao

    void setMapUpdateDistance(float distance) { map_update_distance_ = distance; }
    void setMapUpdateAngle(float angle) { map_update_angle_ = angle; }
    void setSubmapSize(int size) { submap_size_ = size; }

    Eigen::Matrix4f getTransformation() { return pose_; }
    std::vector< std::pair< Eigen::Vector3d, Eigen::Quaterniond> > vPoses; // 将历史位姿添加进去
    void getMap(sensor_msgs::PointCloud2& map_msg)
    {
        //std::unique_lock<std::mutex> locker(map_mutex_);  //
        //pcl::toROSMsg(map_, map_msg);
        pcl::toROSMsg(map__, map_msg);
    }
    void getPoseGraph(std::vector<Eigen::Vector3d>& nodes,
                      std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& edges);

    void doPoseOptimize();
    

private:
    //void downSample(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud,
                    //pcl::PointCloud<pcl::PointXYZI>::Ptr& sampled_cloud);
    void downSample(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud,
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& sampled_cloud);  // TODO @Yimin Zhao

    void updateMap();
    void updateSubmap();
    void detectLoopClosure(const KeyFrame::Ptr& key_frame);
    KeyFrame::Ptr getClosestKeyFrame(const KeyFrame::Ptr& key_frame,
                                     const std::vector<KeyFrame::Ptr>& candidates);
    bool needOptimize();

private:
    //pcl::PointCloud<pcl::PointXYZI> map_;
    pcl::PointCloud<pcl::PointXYZRGB> map_;                       // TODO @Yimin Zhao
    pcl::PointCloud<pcl::PointXYZRGB> map__;                      // TODO @Yimin Zhao
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> submap_;  // TODO @Yimin Zhao
    std::vector<KeyFrame::Ptr> key_frames_;      // keyframe
    std::vector<LoopConstraint> vLoopConstraint; // loop constraint 
    //pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt_;
    pcl::NormalDistributionsTransform<pcl::PointXYZRGB, pcl::PointXYZRGB> ndt_;  // TODO @Yimin Zhao
    ceres::LocalParameterization* quaternion_local_parameterization =
        new ceres::EigenQuaternionParameterization;

    Eigen::Matrix4f pose_;  // New pose 
    Eigen::Matrix4f last_update_pose_; // Last pose
    float voxel_grid_leaf_size_;
    float map_update_distance_;
    float map_update_angle_;
    float loop_search_distance_; 
    float loop_min_fitness_score_; 
    bool enable_optimize_;
    int loop_keyframe_skip_;
    int loop_min_chain_size_; 
    int submap_size_; // Submap size
    int sequence_num_;
    int loop_constraint_count_;
    int optimize_every_n_constraint_;
    std::chrono::steady_clock::time_point optimize_time_;
    bool first_point_cloud_;
    std::mutex map_mutex_; 
    std::mutex vPoses_mutex;
};

} // namespace lidar_slam_3d

#endif // MAP_BUILDER_H
