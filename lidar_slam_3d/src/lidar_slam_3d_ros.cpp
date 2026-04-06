#include "lidar_slam_3d_ros.h"
#include "geo_transform.h"
#include <pcl_conversions/pcl_conversions.h>


LidarSlam3dRos::LidarSlam3dRos()
{
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    std::string point_cloud_topic, gps_topic, imu_topic;

    private_nh.param("base_frame", base_frame_, std::string("base_link"));
    private_nh.param("map_frame", map_frame_, std::string("map"));
    private_nh.param("imu_frame", imu_frame_, std::string("imu"));
    private_nh.param("publish_freq", publish_freq_, 0.5);
    //private_nh.param("point_cloud_topic", point_cloud_topic, std::string("velodyne_points")); 
    private_nh.param("point_cloud_topic", point_cloud_topic, std::string("camera/cloud"));  // TODO @Yimin Zhao
    private_nh.param("gps_topic", gps_topic, std::string("gps"));
    private_nh.param("imu_topic", imu_topic, std::string("imu_raw"));
    private_nh.param("min_scan_distance", min_scan_distance_, 0.002);
    //private_nh.param("enable_floor_filter", enable_floor_filter_, true);
    private_nh.param("enable_floor_filter", enable_floor_filter_, false); // TODO @Yimin Zhao
    private_nh.param("initial_point_cloud", initial_point_cloud_, true);  // TODO @Yimin Zhao
    private_nh.param("speed_stage_rpm", speed_stage_rpm_, 3.3);           // TODO @Yimin Zhao, 1.65rpm -- 500; 3.3rpm -- 1000
    private_nh.param("angle_scale", angle_scale_, 0.0);                   // TODO @Yimin Zhao  angle_scale_ -- modify initial angle, unit degree 

    map_pub_ = nh.advertise<sensor_msgs::PointCloud2>("point_cloud_map", 1, true);
    path_pub_ = nh.advertise<nav_msgs::Path>("path", 1, true);
    gps_path_pub_ = nh.advertise<nav_msgs::Path>("gps_path", 1, true);
    pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("pose", 1, true);
    filtered_point_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("filtered_point_cloud", 1, true);
    filtered_points_pub_ = nh.advertise<sensor_msgs::PointCloud2>("filtered_points", 1, true); // TODO @Yimin Zhao
    floor_points_pub_ = nh.advertise<sensor_msgs::PointCloud2>("floor_points", 1, true);
    constraint_list_pub_ = nh.advertise<visualization_msgs::MarkerArray>("constraint_list", 1, true);

    optimization_srv_ = nh.advertiseService("optimization", &LidarSlam3dRos::optimizationCallback, this);

    point_cloud_sub_ = nh.subscribe(point_cloud_topic, 10000, &LidarSlam3dRos::pointCloudCallback, this);
    gps_sub_ = nh.subscribe(gps_topic, 100, &LidarSlam3dRos::gpsCallback, this);
    imu_sub_ = nh.subscribe(imu_topic, 100, &LidarSlam3dRos::imuCallback, this);
    // Start publishLoop
    publish_thread_.reset(new std::thread(std::bind(&LidarSlam3dRos::publishLoop, this)));
}

// Computer xyzrpy vector from transformation matrix
Vector6f LidarSlam3dRos::getPose(const Eigen::Matrix4f& T)
{
    Vector6f pose;
    pose(0) = T(0, 3);
    pose(1) = T(1, 3);
    pose(2) = T(2, 3);

    tf::Matrix3x3 R;
    double roll, pitch, yaw;
    R.setValue(T(0, 0), T(0, 1), T(0, 2),
               T(1, 0), T(1, 1), T(1, 2),
               T(2, 0), T(2, 1), T(2, 2));
    R.getRPY(roll, pitch, yaw);
    pose(3) = roll;
    pose(4) = pitch;
    pose(5) = yaw;

    return pose; 
}

// Pose optimaztion using map_builder_
bool LidarSlam3dRos::optimizationCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    map_builder_.doPoseOptimize();
    return true;
}

// Draw gps trajectory (refer to vins fusion)
void LidarSlam3dRos::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& gps_msg)
{
    if(gps_msg->status.status == -1) {
        ROS_WARN("Lost Gps!!!");
        return;
    }

    double x, y;
    lidar_slam_3d::WGS84ToUTM(gps_msg->latitude, gps_msg->longitude, x, y);

    Eigen::Vector3d pose(x, y, 0.0);

    if(!gps_origin_) {
        gps_origin_ = pose;
    }
    pose -= *gps_origin_;

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = gps_msg->header.stamp;
    pose_msg.header.frame_id = map_frame_;
    pose_msg.pose.position.x = pose(0);
    pose_msg.pose.position.y = pose(1);
    pose_msg.pose.position.z = pose(2);

    tf::Quaternion q;
    q.setRPY(0.0, 0.0, 0.0);
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();

    gps_path_msg_.poses.push_back(pose_msg);

    gps_path_msg_.header.stamp = gps_msg->header.stamp;
    gps_path_msg_.header.frame_id = map_frame_;
    gps_path_pub_.publish(gps_path_msg_);
}

// imu proccess
void LidarSlam3dRos::imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
    static ros::Time imu_last_ = ros::Time(0);
    float dt = (imu_msg->header.stamp-imu_last_).toSec();
    imu_last_ = imu_msg->header.stamp;
}

// Point cloud match
void LidarSlam3dRos::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg)
{
    if(initial_point_cloud_) {
       initial_point_cloud_ = false;
       initial_ros_time_ =  point_cloud_msg->header.stamp; 
    }
    // time stamp calculation
    time_stamp_ = point_cloud_msg->header.stamp - initial_ros_time_;
    time_stamp = time_stamp_.toSec();

    // angle calculation  
    angle_velocity_ = 2*M_PI*speed_stage_rpm_/60; // red/s;
    angley = angle_velocity_ * time_stamp - angle_scale_*M_PI/180;  // TODO @Yimin Zhao add "angle_scale_"

    std::cout << "angle_velocity_ " << angle_velocity_ << std::endl;
    std::cout << "angley " << angley << std::endl;
    std::cout << "time_stamp " << time_stamp << std::endl;
    
    // ros_msg -> pcl_cloud  
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_m(new pcl::PointCloud<pcl::PointXYZRGB>()); // TODO @Yimin Zhao
    pcl::fromROSMsg(*point_cloud_msg, *point_cloud_m); 
        

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ (new pcl::PointCloud<pcl::PointXYZRGB>); // TODO @Yimin Zhao   
    filter(point_cloud_m, point_cloud_); // filter the the points outside the workspace
    ros::Duration(0.1).sleep();
    
    // transform point cloud from blaze optical coordinate system to rotation stge coordinate system 1
    Eigen::AngleAxisf init_rotation1 (0.0, Eigen::Vector3f::UnitY ());
    //Eigen::Translation3f init_translation1 (2 , -30,  -367); // sysytem calibration
    Eigen::Translation3f init_translation1 (2 , -95,  -398.5); // sysytem calibration 
    Eigen::Matrix4f init_guess1 = (init_translation1 * init_rotation1).matrix ();
    
    // transform point cloud from rotation stge coordinate system 1 to rotation stge coordinate system 2
    Eigen::AngleAxisf init_rotation2 (-M_PI/2, Eigen::Vector3f::UnitX ());
    Eigen::Translation3f init_translation2 (0.0 , 0.0,  0.0); // sysytem calibration 
    Eigen::Matrix4f init_guess2 = (init_translation2 * init_rotation2).matrix ();
    
    // transform point cloud from blaze optical coordinate system to rotation stge coordinate system 2
    Eigen::Matrix4f init_guess = init_guess2*init_guess1;
    
    // transform point cloud from blaze coordiante system to rotation platform stage coordiante system 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud (new pcl::PointCloud<pcl::PointXYZRGB>); // TODO @Yimin Zhao   
    pcl::transformPointCloud (*point_cloud_, *point_cloud, init_guess);

    // Publish 
    sensor_msgs::PointCloud2 filtered_point_cloud;
    pcl::toROSMsg(*point_cloud, filtered_point_cloud);
    filtered_point_cloud.header.stamp = point_cloud_msg->header.stamp;
    filtered_point_cloud.header.frame_id =  point_cloud_msg->header.frame_id; // TODO @Yimin Zhao
    filtered_points_pub_.publish(filtered_point_cloud);

    // Start building map
    if(enable_floor_filter_!) {

          map_builder_.addPointCloud(point_cloud, angley); // TODO @Yimin Zhao   

    }
    
    // Publish Result
    sensor_msgs::PointCloud2 map_msg;
    map_builder_.getMap(map_msg);
    map_msg.header.stamp = point_cloud_msg->header.stamp;
    map_msg.header.frame_id =  point_cloud_msg->header.frame_id; // TODO @Yimin Zhao
    map_pub_.publish(map_msg);

}

// Generate pose and publish
void LidarSlam3dRos::publishPose(const Vector6f& pose, const ros::Time& t)
{
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = t;
    pose_msg.header.frame_id = map_frame_;
    pose_msg.pose.position.x = pose(0);
    pose_msg.pose.position.y = pose(1);
    pose_msg.pose.position.z = pose(2);
    tf::Quaternion q;
    q.setRPY(pose(3), pose(4), pose(5));
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();

    pose_pub_.publish(pose_msg);
}

// Generate path and publish
void LidarSlam3dRos::publishPath()
{
    path_msg_.poses.clear(); // clear path
    geometry_msgs::PoseStamped pose_msg;

    for(auto iter = map_builder_.vPoses.begin();iter!=map_builder_.vPoses.end();iter++){
        pose_msg.header.stamp = ros::Time::now();
        pose_msg.header.frame_id = map_frame_;
        pose_msg.pose.position.x = iter->first.x();
        pose_msg.pose.position.y = iter->first.y();
        pose_msg.pose.position.z = iter->first.z();

        pose_msg.pose.orientation.x = iter->second.x();
        pose_msg.pose.orientation.y = iter->second.y();
        pose_msg.pose.orientation.z = iter->second.z();
        pose_msg.pose.orientation.w = iter->second.w();

        path_msg_.poses.push_back(pose_msg);
    }

    path_msg_.header.stamp = ros::Time::now();
    path_msg_.header.frame_id = map_frame_;
    path_pub_.publish(path_msg_);
}

void LidarSlam3dRos::publishTf(const Vector6f& pose, const ros::Time& t)
{
    // Publish current pose
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(pose(0), pose(1), pose(2)));
    tf::Quaternion q;
    q.setRPY(pose(3), pose(4), pose(5));
    transform.setRotation(q);
    tf_broadcaster_.sendTransform(tf::StampedTransform(transform, t, "map", "base_link"));
}

void LidarSlam3dRos::publishMap()
{
    sensor_msgs::PointCloud2 map_msg;

    map_builder_.getMap(map_msg);
    map_msg.header.stamp = ros::Time::now();
    map_msg.header.frame_id =  "camera_optical_frame"; // TODO @Yimin Zhao
    map_pub_.publish(map_msg);
}


void LidarSlam3dRos::publishConstraintList()
{
    // Generate pose graph
    std::vector<Eigen::Vector3d> graph_nodes;
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> graph_edges;

    map_builder_.getPoseGraph(graph_nodes, graph_edges);

    visualization_msgs::MarkerArray marker_array;

    visualization_msgs::Marker marker;
    marker.header.frame_id = map_frame_;
    marker.header.stamp = ros::Time::now();
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.scale.x = 0.15;
    marker.scale.y = 0.15;
    marker.scale.z = 0.15;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(0);

    visualization_msgs::Marker edge;
    edge.header.frame_id = map_frame_;
    edge.header.stamp = ros::Time::now();
    edge.action = visualization_msgs::Marker::ADD;
    edge.id = 0;
    edge.type = visualization_msgs::Marker::LINE_STRIP;
    edge.scale.x = 0.1;
    edge.scale.y = 0.1;
    edge.scale.z = 0.1;
    edge.color.r = 0.0;
    edge.color.g = 1.0;
    edge.color.b = 0.0;
    edge.color.a = 1.0;

    int id = 0;
    for (int i = 0; i < graph_nodes.size(); ++i) {
        marker.id = id;
        marker.pose.position.x = graph_nodes[i](0);
        marker.pose.position.y = graph_nodes[i](1);
        marker.pose.position.z = graph_nodes[i](2);
        marker_array.markers.push_back(visualization_msgs::Marker(marker));
        id++;
    }

    for (int i = 0; i < graph_edges.size(); ++i) {
        edge.points.clear();
        geometry_msgs::Point p;
        p.x = graph_edges[i].first(0);
        p.y = graph_edges[i].first(1);
        p.z = graph_edges[i].first(2);
        edge.points.push_back(p);
        p.x = graph_edges[i].second(0);
        p.y = graph_edges[i].second(1);
        p.z = graph_edges[i].second(2);
        edge.points.push_back(p);
        edge.id = id;
        marker_array.markers.push_back(visualization_msgs::Marker(edge));
        id++;
    }

    constraint_list_pub_.publish(marker_array);
}


void LidarSlam3dRos::publishLoop()
{
    ros::Rate rate(publish_freq_);

    while (ros::ok()) {
        publishPath();
        publishMap();
        publishConstraintList();
        rate.sleep();
    }
}

// Filter function definition
void LidarSlam3dRos::filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_m, pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filteredx (new pcl::PointCloud<pcl::PointXYZRGB>);  // TODO @Yimin Zhao
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filteredy (new pcl::PointCloud<pcl::PointXYZRGB>);  // TODO @Yimin Zhao
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filteredz (new pcl::PointCloud<pcl::PointXYZRGB>);  // TODO @Yimin Zhao
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);    // TODO @Yimin Zhao
        
    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZRGB> xpass;
    xpass.setInputCloud (point_cloud_m);
    xpass.setFilterFieldName ("x"); //point right and is X axis direction in the stage coordinate system
    //xpass.setFilterLimits (-8, 6);
    //xpass.setFilterLimits (-5, 3);
    //xpass.setFilterLimits (-5, 1);
    xpass.setFilterLimits (-4, 1);
    //xpass.setFilterLimits (-3, 0);
    //xpass.setFilterLimits (-30, 30);
    //xpass.setFilterLimits (-20, 20);
    xpass.setNegative (false);
    xpass.filter (*cloud_filteredx);
    
    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZRGB> ypass;
    ypass.setInputCloud (cloud_filteredx);
    ypass.setFilterFieldName ("y"); // point down and is -Z axis direction in the stage coordinate system
    //ypass.setFilterLimits (-88, 88);
    ypass.setFilterLimits (-88, 95);
    ypass.setNegative (false);
    ypass.filter (*cloud_filteredy);
  
    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZRGB> zpass;
    zpass.setInputCloud (cloud_filteredy);
    zpass.setFilterFieldName ("z"); // point forward and is Y axis direction in the stage coordinate system
    zpass.setFilterLimits (300, 450);
    zpass.setNegative (false);
    zpass.filter (*cloud_filteredz);

    // StatisticalOutlierRemoval
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud (cloud_filteredz);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1);
    sor.filter (*point_cloud);
    return;
}
