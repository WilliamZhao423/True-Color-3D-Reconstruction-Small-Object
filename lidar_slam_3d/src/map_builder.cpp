#include "map_builder.h"
#include <ceres/ceres.h>
#include "pose_graph_error.h"
#include "pcl/io/ply_io.h"

using ceres::AutoDiffCostFunction;
using ceres::Solver;
using ceres::Solve;
using ceres::CostFunction;

namespace lidar_slam_3d
{

// Default construction function
MapBuilder::MapBuilder() :   
    first_point_cloud_(true), sequence_num_(0),
    pose_(Eigen::Matrix4f::Identity()), last_update_pose_(Eigen::Matrix4f::Identity()),
    submap_size_(800), voxel_grid_leaf_size_(1.5), map_update_distance_(1), enable_optimize_(true),
    loop_search_distance_(20), loop_min_chain_size_(5), loop_min_fitness_score_(1.5),
    loop_keyframe_skip_(20), loop_constraint_count_(0), optimize_every_n_constraint_(10)
{
    // ndt algorithm parameters
    ndt_.setTransformationEpsilon(1e-3); // TODO
    ndt_.setStepSize(0.2);
    ndt_.setResolution(2);
    ndt_.setMaximumIterations(30);

}

// Sample with voxelGridFilter
void MapBuilder::downSample(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud,
                            pcl::PointCloud<pcl::PointXYZRGB>::Ptr& sampled_cloud)   // TODO @Yimin Zhao                         
{
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid_filter;   // TODO @Yimin Zhao
    voxel_grid_filter.setLeafSize(voxel_grid_leaf_size_, voxel_grid_leaf_size_, voxel_grid_leaf_size_);
    voxel_grid_filter.setInputCloud(input_cloud);
    voxel_grid_filter.filter(*sampled_cloud);
}


KeyFrame::Ptr MapBuilder::getClosestKeyFrame(const KeyFrame::Ptr& key_frame,
                                             const std::vector<KeyFrame::Ptr>& candidates)
{
    Eigen::Vector3d pt1 = key_frame->getPose().block<3, 1>(0, 3).template cast<double>();
    float min_distance = std::numeric_limits<float>::max();
    int id;

    for(const KeyFrame::Ptr& frame : candidates) {
        Eigen::Vector3d pt2 = frame->getPose().block<3, 1>(0, 3).template cast<double>();
        float distance = (pt1 - pt2).norm();
        if(distance < min_distance) {
            min_distance = distance;
            id = frame->getId();
        }
    }

    return key_frames_[id];
}

// Loop detection
void MapBuilder::detectLoopClosure(const KeyFrame::Ptr& key_frame)
{
    std::vector<KeyFrame::Ptr> cloud_chain;
    std::vector<std::vector<KeyFrame::Ptr>> cloud_chains;

    // Extract pose information
    int n = key_frames_.size();
    Eigen::Vector3d pt1 = key_frame->getPose().block<3, 1>(0, 3).template cast<double>();


    for(int i = 0; i < n; ++i) {
        Eigen::Vector3d pt2 = key_frames_[i]->getPose().block<3, 1>(0, 3).template cast<double>();
        float distance = (pt1 - pt2).norm();

        if(distance < loop_search_distance_) {
            if(key_frames_[i]->getId() < key_frame->getId() - loop_keyframe_skip_) {
                cloud_chain.push_back(key_frames_[i]);
            }
            else {
                cloud_chain.clear();
            }
        }

        else {
            if(cloud_chain.size() > loop_min_chain_size_) {
                cloud_chains.push_back(cloud_chain);
                std::cout << "\033[36m" << "Find loop candidates. Keyframe chain size "
                          << cloud_chain.size() << std::endl;
                cloud_chain.clear();
            }
            else {
                cloud_chain.clear(); 
            }
        }

    }

    if(cloud_chains.empty()) {
        return;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sampled_cloud(new pcl::PointCloud<pcl::PointXYZRGB>()); // TODO @Yimin Zhao
    downSample(key_frame->getCloud(), sampled_cloud);


    for(const std::vector<KeyFrame::Ptr>& chain : cloud_chains) {
    
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());  // TODO @Yimin Zhao

        // Build local map
        for(const KeyFrame::Ptr& frame : chain) {
            pcl::PointCloud<pcl::PointXYZRGB> transformed_cloud; // TODO @Yimin Zhao
            pcl::transformPointCloud(*(frame->getCloud()), transformed_cloud, frame->getPose());
            *target_cloud += transformed_cloud;
        }

        // match keyframe point cloud with local map (ndt algorithm)
        pcl::PointCloud<pcl::PointXYZRGB> output_cloud;   // TODO @Yimin Zhao
        pcl::NormalDistributionsTransform<pcl::PointXYZRGB, pcl::PointXYZRGB> ndt;   // TODO @Yimin Zhao
        
        ndt.setTransformationEpsilon(1e-3);    // TODO @Yimin Zhao
        ndt.setStepSize(0.2);                // TODO @Yimin Zhao
        ndt.setResolution(2);              // TODO @Yimin Zhao
        ndt.setMaximumIterations(30);
        ndt.setInputSource(sampled_cloud); 
        ndt.setInputTarget(target_cloud);
        ndt.align(output_cloud, key_frame->getPose());

        Eigen::Matrix4f loop_pose = ndt.getFinalTransformation();

        bool converged = ndt.hasConverged();
        double fitness_score = ndt.getFitnessScore();
        int final_num_iteration = ndt.getFinalNumIteration();

        std::cout << "Loop registration fitness_score " << fitness_score << std::endl;

        // If match is successful
        if(converged && fitness_score < loop_min_fitness_score_) {

            KeyFrame::Ptr closest_keyframe = getClosestKeyFrame(key_frame, chain);

            LoopConstraint lc; lc.first_ = key_frame->getId(); lc.second_ = closest_keyframe->getId();
            lc.loop_pose_ = loop_pose;
            vLoopConstraint.push_back(lc);

            loop_constraint_count_++;
            optimize_time_ = std::chrono::steady_clock::now();
            std::cout << "Add loop constraint." << std::endl;
        }
    }
}


bool MapBuilder::needOptimize()
{
    if(loop_constraint_count_ > optimize_every_n_constraint_) {
        return true;
    }

    if(loop_constraint_count_ > 0) {
        auto delta_t = std::chrono::duration_cast<std::chrono::duration<double>>(
                       std::chrono::steady_clock::now() - optimize_time_);
        if(delta_t.count() > 10.0) {
            return true;
        }
    }

    return false;
}


void MapBuilder::addPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& point_cloud, double angley) // TODO
{
    if((*point_cloud).size()==0)
    {
                PCL_ERROR ("Couldn't read target file  \n");
                return ;
    }
    
    // guess transformation
    Eigen::AngleAxisf guess_rotation (-angley, Eigen::Vector3f::UnitZ ());
    Eigen::Translation3f guess_translation (0 , 0,  0);
    Eigen::Matrix4f guess_transform = (guess_translation * guess_rotation).matrix ();
    // target point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cloud (new pcl::PointCloud<pcl::PointXYZRGB>); // TODO
         
    auto t1 = std::chrono::steady_clock::now();
    sequence_num_++; 
    std::cout<<"sequence_num_: "<<sequence_num_ <<std::endl;  

    if(first_point_cloud_) {
        first_point_cloud_ = false;        
        map_ += *point_cloud;
        map__+= *point_cloud;
        submap_.push_back(point_cloud);

        KeyFrame::Ptr key_frame(new KeyFrame());
        key_frame->setId(key_frames_.size());
        
        key_frame->setPose(pose_);
        key_frame->setCloud(point_cloud);
        key_frames_.push_back(key_frame);
        //std::cout << "\033[1m\033[32m" << "------ Insert keyframe " << key_frames_.size() << " ------" << std::endl;
        return;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sampled_cloud(new pcl::PointCloud<pcl::PointXYZRGB>); // TODO @Yimin Zhao
    downSample(point_cloud, sampled_cloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZRGB>); // TODO @Yimin Zhao
    
    // Transforming point_cloud using guess transform
    pcl::transformPointCloud (*point_cloud, *target_cloud, guess_transform); 
    
    double angle = 180*angley/M_PI;
    int ang = int(angle);
    std::cout << "angle" << angle << std::endl;
    std::cout << "ang" << ang << std::endl;

    if (ang % 1 == 0 )
    {
        //submap_.push_back(output_cloud);
        submap_.push_back(target_cloud);
    }
    
    while(submap_.size() > submap_size_) {
        submap_.erase(submap_.begin());
    }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr map_cloud(new pcl::PointCloud<pcl::PointXYZRGB>); // TODO @Yimin Zhao 
    map_cloud->clear();
    
    for(int i = 0; i < submap_.size(); ++i) {
           *map_cloud += *submap_[i];
    }    
    map__ = *map_cloud;
    
    std::cout << "submap_.size()" << submap_.size() << std::endl;
    std::cout << "map__.size()" << map_cloud->size() << std::endl;
    

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr map_sampled_cloud(new pcl::PointCloud<pcl::PointXYZRGB>); // TODO @Yimin Zhao
     
    if(ang > 365){ // Small object

        downSample(map_cloud, map_sampled_cloud);

        exit(0);
    }
    

// Update map
void MapBuilder::updateMap()
{
    std::cout << "Start update map ..." << std::endl;
    std::unique_lock<std::mutex> lock(map_mutex_);

    map_.clear(); // Point cloud type
    submap_.clear();

    int n = key_frames_.size();
    for(int i = 0; i < n; ++i) {
        // Add point cloud to map
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());  // TODO @Yimin Zhao
        pcl::transformPointCloud(*(key_frames_[i]->getCloud()), *transformed_cloud, key_frames_[i]->getPose());
        map_ += *transformed_cloud;

        // Add latest submap_size_ point clouds to local map
        if(i > n - submap_size_) {
            submap_.push_back(transformed_cloud);
        }
    }
    std::cout << "Finish update map." << std::endl;
}

// Pose optimization
void MapBuilder::doPoseOptimize()
{   
    ceres::Problem problem;
    ceres::Solver::Options options; // ceres solver
    ceres::Solver::Summary summary;
    ceres::LocalParameterization* quaternion_local_parameterization = new ceres::QuaternionParameterization();
    // initial ceres
    options.max_num_iterations = 1000;
    // options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = false;

    int kf_len = key_frames_.size(); // keyframe length
    Eigen::Vector3d array_t[kf_len];
    Eigen::Quaterniond array_q[kf_len]; // restore quaternion
    array_t[0] = key_frames_[0]->getPosePQ().p;
    array_q[0] = key_frames_[0]->getPosePQ().q;

    // Add pose in key_frames_
    for(int i = 1;i<kf_len-1;i++){

        Eigen::Quaterniond source_q, target_q;
        Eigen::Vector3d source_p, target_p;

        array_t[i] = key_frames_[i]->getPosePQ().p;
        array_q[i] = key_frames_[i]->getPosePQ().q;

        Eigen::Matrix4f source_pose = key_frames_[i]->getPose();
        Eigen::Matrix4f target_pose = key_frames_[i-1]->getPose();
        source_p = source_pose.block<3,1>(0,3).template cast<double>();
        target_p = target_pose.block<3,1>(0,3).template cast<double>();
        source_q = Eigen::Quaterniond(source_pose.block<3,3>(0,0).template cast<double>());
        target_q = Eigen::Quaterniond(target_pose.block<3,3>(0,0).template cast<double>());

        PosePQ relative_measured;
        Eigen::Quaterniond source_q_inv = source_q.conjugate();
        Eigen::Quaterniond relative_q = source_q_inv * target_q;
        Eigen::Vector3d relative_p = source_q_inv*(target_p - source_p);
        relative_measured.p = relative_p;
        relative_measured.q = relative_q;

        ceres::LossFunction* loss_function = NULL;
        ceres::CostFunction* cost_function = PoseGraph3dErrorTerm::Create(relative_measured);

        problem.AddResidualBlock(cost_function, loss_function, 
            array_t[i].data(), array_q[i].coeffs().data(), array_t[i-1].data(), array_q[i-1].coeffs().data());

        problem.SetParameterization(array_q[i].coeffs().data(),
                                 quaternion_local_parameterization);
    }

    // Fix the first pose (no optimization)
    problem.SetParameterBlockConstant(array_t[0].data()); 
    problem.SetParameterBlockConstant(array_q[0].coeffs().data());
    problem.SetParameterization(array_q[0].coeffs().data(),
                                 quaternion_local_parameterization);
    
    // Add pose constraint
    int loop_size = vLoopConstraint.size();
    for(int i = 0; i<loop_size; i++){
        Eigen::Quaterniond source_q, target_q;
        Eigen::Vector3d source_p, target_p;

        Eigen::Matrix4f source_pose = vLoopConstraint[i].loop_pose_;
        Eigen::Matrix4f target_pose = key_frames_[vLoopConstraint[i].second_]->getPose();
        source_p = source_pose.block<3,1>(0,3).template cast<double>();
        target_p = target_pose.block<3,1>(0,3).template cast<double>();
        source_q = Eigen::Quaterniond(source_pose.block<3,3>(0,0).template cast<double>());
        target_q = Eigen::Quaterniond(target_pose.block<3,3>(0,0).template cast<double>());

        // Compute motion between pq
        PosePQ relative_measured;
        Eigen::Quaterniond source_q_inv = source_q.conjugate();
        Eigen::Quaterniond relative_q = source_q_inv * target_q;
        Eigen::Vector3d relative_p = source_q_inv*(source_p - target_p);
        relative_measured.p = relative_p;
        relative_measured.q = relative_q;

        ceres::LossFunction* loss_function = NULL;
        ceres::CostFunction* cost_function = PoseGraph3dErrorTerm::Create(relative_measured);

        problem.AddResidualBlock(cost_function, loss_function, 
            array_t[vLoopConstraint[i].first_].data(), array_q[vLoopConstraint[i].first_].coeffs().data(), 
                            array_t[vLoopConstraint[i].second_].data(), array_q[vLoopConstraint[i].second_].coeffs().data());

    }

    std::cout<<"\033[33mOptimizing....\033[0m"<<std::endl;
    ceres::Solve(options, &problem, &summary);
    
    if(!summary.IsSolutionUsable())
        std::cout << "CERES SOLVE FAILED" <<std::endl;
    else{
        std::cout<< summary.BriefReport() <<std::endl;
        std::unique_lock<std::mutex> locker(vPoses_mutex);
        for(int i = 0; i<key_frames_.size(); i++){
            key_frames_[i]->setPosePQ(array_t[i],array_q[i]);
            vPoses.push_back(std::make_pair(array_t[i],array_q[i]));
        }
    }
}


void MapBuilder::getPoseGraph(std::vector<Eigen::Vector3d>& nodes,
                              std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& edges)
{  
    return;
}

}
