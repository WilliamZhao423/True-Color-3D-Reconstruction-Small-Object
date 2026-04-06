/**
 *  Copyright (c) Hofsta University  - All Rights Reserved
 *  Created on: @26/02/2023
 *  Author: Yimin.Zhao <yimin.zhao@hofstra.edu>
 */ 

// Include ROS
#include <ros/ros.h> 
#include <sensor_msgs/PointCloud2.h> 
#include <sensor_msgs/Image.h> 
#include <camera_info_manager/camera_info_manager.h> 
// Suppress warnings coming from PCL headers.
#pragma warning(push, 1)
#pragma warning(disable : 4068) // unknown pragma
#pragma warning(disable : 4702) // unreachable code
#pragma push_macro("BOOST_ALLOW_DEPRECATED_HEADERS")
#ifndef BOOST_ALLOW_DEPRECATED_HEADERS
#    define BOOST_ALLOW_DEPRECATED_HEADERS
#endif
#include <map>
#pragma GCC diagnostic push
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
// Include PCL
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h> // 
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_ros/point_cloud.h> //
#pragma GCC diagnostic pop
#pragma pop_macro("BOOST_ALLOW_DEPRECATED_HEADERS")
#pragma warning(pop)
// Include OpenCV
#if CV_VERSION_MAJOR > 2
#    include <opencv2/calib3d.hpp>
#    include <opencv2/core.hpp>
#    include <opencv2/highgui.hpp>
#    include <opencv2/imgproc.hpp>
#else
#    include <opencv2/core/core.hpp>
#    include <opencv2/highgui/highgui.hpp>
#    include <opencv2/imgproc/imgproc.hpp>
#    include <opencv2/calib3d/calib3d.hpp>
#endif
// Include files to use the pylon API
#include <pylon/PylonIncludes.h>
#pragma warning(push)
#pragma warning(disable : 4702) // C4702: unreachable code
#include <pylon/BlazeInstantCamera.h>
#include <pylon/BaslerUniversalInstantCamera.h>
#pragma warning(pop) 

// Namespaces for using the pylon API and the camera parameters
using namespace Pylon;
namespace blazeParams = BlazeCameraParams_Params;
namespace colorParams = Basler_UniversalCameraParams;

// Type defs for the PCL types used
typedef pcl::PointXYZRGB Point_t;
typedef pcl::PointCloud<Point_t> PointCloud_t;
typedef PointCloud_t::Ptr PointCloudPtr;

class Fusion
{
public:
    int run();
    void saveDataset(const PointCloudPtr& pointcloud);
    void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void*);
    void initCloudViewer(pcl::visualization::PCLVisualizer& viewer);    
    // Topics
    ros::Publisher m_cloudPub; //

private:
    void loadCalibrationFile();
    void setupBlazeCamera();
    void setupColorCamera();
    cv::Mat processBlazeData(const CGrabResultPtr& result);
    cv::Mat processColorData(const CGrabResultPtr& result);
    std::vector<bool> locateOccludedPoints(const cv::Mat& pointcloudVec,
                                           const cv::Mat& projectedPoints,
                                           const cv::Size colorSize);
    cv::Mat warpColorToDepth(const cv::Mat& pointcloud, const cv::Mat& color);
    // Publish results.
    bool publish(const PointCloudPtr& pointCloud, ros::Time acquisition_time);

private:
    CBlazeInstantCamera m_blazeCamera;
    CBaslerUniversalInstantCamera m_colorCamera;
    cv::Mat m_rotation;
    cv::Mat m_translation;
    cv::Mat m_colorCameraMatrix;
    cv::Mat m_colorDistortion;
    
    // pylon image format converter
    CImageFormatConverter m_converter;

    int m_savedImageCnt = 0;
    bool m_saveImage = false;
};

std::string g_frameId; 
// Node features
const std::string node_name = "basler_blaze_fusion_node";

void Fusion::loadCalibrationFile()
{
    // It is assumed that the 'calibration.xml' contains information about the relative orientation
    // of the cameras to each other and the optical calibration of the color camera.
    // The calibration program can be used to create the file.
    std::string filename = std::string("calibration_") + m_blazeCamera.GetDeviceInfo().GetSerialNumber().c_str() + "_" +
                           m_colorCamera.GetDeviceInfo().GetSerialNumber().c_str() + ".xml";
    const std::string path = std::string(CALIBRATION_DIR) + filename;
    std::cout << "Loading the calibration file: " << path << std::endl;
    cv::FileStorage fs(path, cv::FileStorage::READ);

    if (!fs.isOpened())
    {
        std::string msg = "For the fusion of the data, calibration must be performed first.\nMake sure that there is a "
                          "valid calibration with file name '" +
                          filename + "' in the directory.";
        throw RUNTIME_EXCEPTION(msg);
    }

    fs["rotation"] >> m_rotation;
    fs["translation"] >> m_translation;
    fs["colorCameraMatrix"] >> m_colorCameraMatrix;
    fs["colorDistortion"] >> m_colorDistortion;
}

void Fusion::setupBlazeCamera()
{
    // Open the blaze camera, i.e., establish a connection to the camera device.
    // Register a configuration that applies a default setup for blaze cameras.
    m_blazeCamera.Attach(
        CTlFactory::GetInstance().CreateFirstDevice(CDeviceInfo().SetDeviceClass(BaslerGenTlBlazeDeviceClass)));
    m_blazeCamera.RegisterConfiguration(new Pylon::CBlazeDefaultConfiguration(),
                                        Pylon::RegistrationMode_ReplaceAll,
                                        Pylon::Cleanup_Delete);
    m_blazeCamera.Open();

    std::cout << "Connected to blaze camera " << m_blazeCamera.GetDeviceInfo().GetFriendlyName() << std::endl;

    // Enable depth data.
    m_blazeCamera.ComponentSelector.SetValue(blazeParams::ComponentSelector_Range);
    m_blazeCamera.ComponentEnable.SetValue(true);
    m_blazeCamera.PixelFormat.SetValue(blazeParams::PixelFormat_Coord3D_ABC32f);
    m_blazeCamera.Scan3dCoordinateSelector.SetValue(blazeParams::Scan3dCoordinateSelector_CoordinateA);
    m_blazeCamera.Scan3dInvalidDataValue.SetValue(0.0);
    m_blazeCamera.Scan3dCoordinateSelector.SetValue(blazeParams::Scan3dCoordinateSelector_CoordinateB);
    m_blazeCamera.Scan3dInvalidDataValue.SetValue(0.0);
    m_blazeCamera.Scan3dCoordinateSelector.SetValue(blazeParams::Scan3dCoordinateSelector_CoordinateC);
    m_blazeCamera.Scan3dInvalidDataValue.SetValue(0.0);

    // Enable the intensity image.
    m_blazeCamera.ComponentSelector.SetValue(blazeParams::ComponentSelector_Intensity);
    m_blazeCamera.ComponentEnable.SetValue(true);
    m_blazeCamera.PixelFormat.SetValue(blazeParams::PixelFormat_Mono16);

    // Disable the confidence map.
    m_blazeCamera.ComponentSelector.SetValue(blazeParams::ComponentSelector_Confidence);
    m_blazeCamera.ComponentEnable.SetValue(false);

    // Set the exposure time.
    //m_blazeCamera.ExposureTime.SetValue(1000.0);
    m_blazeCamera.ExposureTime.SetValue(100.0); // TODO @2022/09/08 

    // Configure the camera for software triggering.
    m_blazeCamera.TriggerMode.SetValue(blazeParams::TriggerMode_On);
    m_blazeCamera.TriggerSource.SetValue(blazeParams::TriggerSource_Software);
}

void Fusion::setupColorCamera()
{
    // Since the blaze camera isn't listed as a GigE device, the first GigE device is opened here as a color camera.
    m_colorCamera.Attach(
        CTlFactory::GetInstance().CreateFirstDevice(CDeviceInfo().SetDeviceClass(BaslerGigEDeviceClass)));
    m_colorCamera.Open();

    // Print the model name of the camera.
    std::cout << "Connected to color camera " << m_colorCamera.GetDeviceInfo().GetFriendlyName() << std::endl;

    // Figure out which 8-bit color pixel format the camera supports.
    // If the camera doesn't support an 8-bit color format, use Mono8 instead.
    GenApi::StringList_t lstPixelFormats; // All pixel formats the camera supports.
    m_colorCamera.PixelFormat.GetSymbolics(lstPixelFormats);
    String_t pixelFormat("Mono8");
    for (auto entry = lstPixelFormats.begin(); entry != lstPixelFormats.end(); ++entry)
    {
        const EPixelType pixelType = CPixelTypeMapper::GetPylonPixelTypeByName(*entry);
        if (IsColorImage(pixelType) && BitPerPixel(pixelType) == 8)
        {
            pixelFormat = *entry;
            break;
        }
    }
    std::cout << "Setting pixel format to " << pixelFormat << std::endl;
    m_colorCamera.PixelFormat.SetValue(pixelFormat);

    // Set the exposure time.
    m_colorCamera.ExposureAuto.SetValue(colorParams::ExposureAuto_Off);
    const float exposureTime = 15000.0f;
    //const float exposureTime = 10000.0f; //TODO @2023/02/24 Yimin Zhao
    if (m_colorCamera.GetSfncVersion() >= Sfnc_2_0_0)
    {
        // ace 2, boost, and dart IMX Cameras
        m_colorCamera.ExposureTime.SetValue(exposureTime);
    }
    else
    {
        // ace classic/U/L GigE Cameras
        m_colorCamera.ExposureTimeAbs.SetValue(exposureTime);
    }

    // If your network hardware can't handle the incoming packet rate, it is
    // useful to increase the delay between packet transmissions.
    m_colorCamera.GevSCPD.SetValue(10000);

    // Increasing the packet size for the color camera will reduce network load.
    // However, for packet sizes above 1500 bytes you have to enable so-called 
    // jumbo frames on your network interface adapter. Basler recommends enabling 
    // jumbo frames in your network adapter's configuration and to uncomment the 
    // following line. Make sure that you enable jumbo frames on your network switch as well!
    // m_colorCamera.GevSCPSPacketSize.SetValue(6000);

    // White balance is adjusted continuously while images are being acquired.
    m_colorCamera.BalanceWhiteAuto.TrySetValue(colorParams::BalanceWhiteAuto_Continuous);

    // Configure the camera for software triggering.
    // Each software trigger will start the acquisition of one single frame.
    m_colorCamera.TriggerSelector.SetValue(colorParams::TriggerSelector_FrameStart);
    m_colorCamera.TriggerMode.SetValue(colorParams::TriggerMode_On);
    m_colorCamera.TriggerSource.SetValue(colorParams::TriggerSource_Software);
}

cv::Mat Fusion::processBlazeData(const CGrabResultPtr& result)
{
    auto container = result->GetDataContainer();
    auto rangeComponent = container.GetDataComponent(0);

    const int width = rangeComponent.GetWidth();
    const int height = rangeComponent.GetHeight();

    cv::Mat pointcloud = cv::Mat(height, width, CV_32FC3, (void*)rangeComponent.GetData()).clone();

    return pointcloud;
}

cv::Mat Fusion::processColorData(const CGrabResultPtr& result)
{
    const auto width = result->GetWidth();
    const auto height = result->GetHeight();

    // Allocate an OpenCV image. 3 byte per pixel, BGR format.
    cv::Mat color((int)height, (int)width, CV_8UC3);

    // Convert the grabbed data to BGR8 format.
    m_converter.Convert(color.ptr(), color.elemSize() * width * height, result);

    return color;
}

std::vector<bool> Fusion::locateOccludedPoints(const cv::Mat& pointcloudVec,
                                               const cv::Mat& projectedPoints,
                                               const cv::Size colorSize)
{
    // The depth camera and the color camera don't have the same optical center.
    // This can cause occlusion problems when mapping the color data.
    // Such effects can be eliminated, for example, by an additional processing step
    // that uses the z-buffer algorithm (https://en.wikipedia.org/wiki/Z-buffering).
    // The z-buffer algorithm saves the depth values for each pixel of the color image.
    // If a pixel of the color image is assigned to multiple pixels of the depth image,
    // only the pixel closest to the camera is colored.

    int numPoints = pointcloudVec.cols;
    std::vector<bool> isOccludedPoint(numPoints, false);
    cv::Mat zBuffer = cv::Mat::zeros(colorSize, CV_32FC1);
    cv::Mat colorIdxBuffer = cv::Mat::zeros(colorSize, CV_32SC1);
    cv::Rect colorRect(cv::Point(), colorSize);
    for (int idx = 0; idx < numPoints; ++idx)
    {
        const cv::Vec3f& point3d = pointcloudVec.at<cv::Vec3f>(idx);
        if (point3d[2] != 0.0f)
        {
            const cv::Point2f& colorPoint = projectedPoints.at<cv::Point2f>(idx);
            if (colorRect.contains(colorPoint))
            {
                auto& z = zBuffer.at<float>(colorPoint);
                auto& colorIndex = colorIdxBuffer.at<int>(colorPoint);
                if (z == 0.0f)
                {
                    // First appearance of the color pixel
                    z = point3d[2];   // Assign depth
                    colorIndex = idx; // Assign index
                }
                else
                {
                    // Reappearance of the color pixel
                    if (z <= point3d[2])
                    {
                        // If the depth in the z-buffer is smaller than the current depth,
                        // the current point is occluded.
                        isOccludedPoint[idx] = true;
                    }
                    else
                    {
                        // If the depth in the z-buffer is greater than the current
                        // depth, the point stored in the z-buffer is occluded.
                        isOccludedPoint[colorIndex] = true;
                        z = point3d[2];   // Assign depth
                        colorIndex = idx; // Assign index
                    }
                }
            }
        }
    }

    return isOccludedPoint;
}

cv::Mat Fusion::warpColorToDepth(const cv::Mat& pointcloud, const cv::Mat& color)
{
    const int width = pointcloud.cols;
    const int height = pointcloud.rows;
    cv::Mat coloredDepth(pointcloud.size(), CV_8UC3);

    // Project the 3D points into the color camera.
    cv::Mat pointcloudVec = pointcloud.reshape(3, 1);
    cv::Mat projectedPoints;
    cv::projectPoints(pointcloudVec,
                      m_rotation,
                      m_rotation * m_translation,
                      m_colorCameraMatrix,
                      m_colorDistortion,
                      projectedPoints);

    // Determine all occluded 3D points
    std::vector<bool> isOccludedPoint = locateOccludedPoints(pointcloudVec, projectedPoints, color.size());

    // Coloring the point cloud
    const cv::Vec3b invalidPoint(0, 0, 0);
    cv::Rect colorRect(cv::Point(), color.size());
    for (int row = 0; row < height; ++row)
    {
        for (int col = 0; col < width; ++col)
        {
            const int idx = row * width + col;
            const cv::Vec3f& point3d = pointcloudVec.at<cv::Vec3f>(idx);
            if (point3d[2] != 0.0f)
            {
                if (!isOccludedPoint[idx])
                {
                    const cv::Point2f& colorPoint = projectedPoints.at<cv::Point2f>(idx);
                    if (colorRect.contains(colorPoint))
                    {
                        coloredDepth.at<cv::Vec3b>(row, col) = color.at<cv::Vec3b>(colorPoint);
                    }
                    else
                    {
                        // No color information available.
                        coloredDepth.at<cv::Vec3b>(row, col) = invalidPoint;
                    }
                }
                else
                {
                    // Occluded point. No color information available.
                    coloredDepth.at<cv::Vec3b>(row, col) = invalidPoint;
                }
            }
            else
            {
                // No depth information available for this pixel. Zero it.
                coloredDepth.at<cv::Vec3b>(row, col) = invalidPoint;
            }
        }
    }

    return coloredDepth;
}

bool Fusion::publish(const PointCloudPtr &PointCloud, ros::Time acquisitionTime)
{
   PointCloud->header.frame_id = g_frameId;
   PointCloud->header.stamp = pcl_conversions::toPCL(acquisitionTime);
   m_cloudPub.publish(*PointCloud);
}

int Fusion::run()
{
    int result = EXIT_SUCCESS;

    // Prepare the pylon image format converter used for converting
    // grabbed images to BGR images to be displayed by OpenCV.
    m_converter.OutputPixelFormat = PixelType_BGR8packed;

    try
    {
        // Smart pointer for accessing the cloud viewer.
        std::shared_ptr<pcl::visualization::CloudViewer> viewer;
        auto isViewerInitialized = std::make_shared<bool>(false); // to be shared with lambda function below

        // Open and configure the cameras.
        setupBlazeCamera();
        setupColorCamera();

        // Load camera calibration.
        loadCalibrationFile();

        // The MaxNumBuffer parameter can be used to control the number of buffers
        // allocated for grabbing. The default value of this parameter is 10.
        m_blazeCamera.MaxNumBuffer = 2;
        m_colorCamera.MaxNumBuffer = 2;

        // Start the acquisition on the host. Now, the enqueued buffers will be filled with data sent by the cameras.
        m_blazeCamera.StartGrabbing();
        m_colorCamera.StartGrabbing();

        // The cameras run in software trigger mode. They will expose and transfer a new image for
        // every software trigger they receive.
        // To optimize bandwidth usage, the color camera is triggered first to
        // allow it to already transfer image data while the blaze camera is still internally
        // processing the acquired raw data.
        m_colorCamera.ExecuteSoftwareTrigger(); // Issue software trigger for the color camera.
        m_blazeCamera.ExecuteSoftwareTrigger(); // Issue software trigger for the blaze camera.

        std::cout << std::endl << "Fusion of color and depth data" << std::endl;
        std::cout << "  - Press 's' to save the current point cloud and image data" << std::endl;
        std::cout << "  - Press 'q' in the viewer to exit" << std::endl;
        std::cout << std::endl;

        // Grab loop. Acquire and process data from both cameras.
        // While the data is being processed for each camera, a new buffer will be filled in
        // the background.
        while (true)
        {

            // Create the point cloud viewer and show the point cloud.
            if (!viewer)
            {
                viewer = std::make_shared<pcl::visualization::CloudViewer>("Basler AG - RGBD-Fusion");
            }

            // Exit if the viewer is closed.
            if (viewer->wasStopped())
            {
                break;
            }

            // Wait for an image and then retrieve it. A timeout of 1000 ms is used.
            CGrabResultPtr ptrColorResult;
            CGrabResultPtr ptrBlazeResult;
            m_colorCamera.RetrieveResult(1000, ptrColorResult, TimeoutHandling_Return);
            m_blazeCamera.RetrieveResult(1000, ptrBlazeResult, TimeoutHandling_Return);

            // Issue software triggers.
            // Wait until the camera is ready to receive a trigger to avoid overtriggering it.
            m_colorCamera.WaitForFrameTriggerReady(1000);
            m_colorCamera.ExecuteSoftwareTrigger(); // Issue software trigger for the blaze camera.
            m_blazeCamera.ExecuteSoftwareTrigger(); // Issue software trigger for the color camera.

            // While the next buffers are being acquired, process the current ones.
            if (!ptrColorResult || !ptrColorResult->GrabSucceeded())
            {
                std::cerr << "Failed to grab image from color camera." << std::endl;
                result = EXIT_FAILURE;
                continue;
            }
            if (!ptrBlazeResult || !ptrBlazeResult->GrabSucceeded())
            {
                std::cerr << "Failed to grab image from blaze camera." << std::endl;
                result = EXIT_FAILURE;
                continue;
            }

            // Process the data.
            cv::Mat colorImg = processColorData(ptrColorResult);
            cv::Mat blazeImg = processBlazeData(ptrBlazeResult);
            cv::Mat coloredDepth = warpColorToDepth(blazeImg, colorImg);

            // Generation of a colored point cloud from the color image
            // warped into the depth image and the point cloud.
            const int width = blazeImg.cols;
            const int height = blazeImg.rows;

            // Convert the grab result to an organized PCL point cloud.
            PointCloudPtr ptrPointCloud(new PointCloud_t);
            ptrPointCloud->width = width;
            ptrPointCloud->height = height;
            ptrPointCloud->points.resize(width * height);
            ptrPointCloud->is_dense = false; // Organized point cloud

            // Set the points.
            for (int row = 0; row < height; row += 1)
            {
                for (int col = 0; col < width; col += 1)
                {
                    auto& coloredPoint = ptrPointCloud->at(col, row); // PCL uses column/row instead of row/column.

                    const cv::Vec3f point3d = blazeImg.at<cv::Vec3f>(row, col);
                    coloredPoint.x = point3d[0];
                    coloredPoint.y = point3d[1];
                    coloredPoint.z = point3d[2];

                    const cv::Vec3b color = coloredDepth.at<cv::Vec3b>(row, col);
                    coloredPoint.r = color[2];
                    coloredPoint.g = color[1];
                    coloredPoint.b = color[0];
                }
            }

            // Show the point cloud.
            viewer->showCloud(ptrPointCloud);

            
            // After showing a point cloud for the first time, it is safe
            // to adjust the settings for rendering the point cloud.
            if (!*isViewerInitialized)
            {
                *isViewerInitialized = true;
                viewer->runOnVisualizationThreadOnce(std::bind(&Fusion::initCloudViewer, this, std::placeholders::_1));
            }

            // Saving the image data if requested.
            if (m_saveImage)
            {
                saveDataset(ptrPointCloud);
                m_saveImage = false;
            }

                publish(ptrPointCloud, ros::Time::now());    

        }
    }
    catch (const GenICam::GenericException& e)
    {
        std::cerr << "Exception occurred: " << e.GetDescription() << std::endl;
        result = EXIT_FAILURE;
    }

    // Shut down acquisition. Stops cameras and frees all grab-related resources.
    if (m_colorCamera.IsOpen())
    {
        try
        {
            // Shut down acquisition. Stops cameras and frees all grab-related resources.
            m_colorCamera.StopGrabbing();
            // Disable software trigger.
            m_colorCamera.TriggerMode.SetValue(colorParams::TriggerMode_Off);
            // Clean-up
            m_colorCamera.Close();
        }
        catch (...)
        {
        }
    }
    if (m_blazeCamera.IsOpen())
    {
        try
        {
            // Shut down acquisition. Stops cameras and frees all grab-related resources.
            m_blazeCamera.StopGrabbing();
            // Disable software trigger.
            m_blazeCamera.TriggerMode.SetValue(blazeParams::TriggerMode_Off);
            // Clean-up
            m_blazeCamera.Close();
        }
        catch (...)
        {
        }
    }
        
    return result;
}

void Fusion::saveDataset(const PointCloudPtr& cloud)
{
    // For point clouds saved as PLY files, the coordinate system is transformed
    // (rotated by 180 degree around the x axis to invert the z axis) by default
    // to meet usual point cloud viewer conventions.
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate(Eigen::AngleAxisf((float)M_PI, Eigen::Vector3f::UnitX()));
    PointCloudPtr transformed_cloud(new PointCloud_t);
    pcl::transformPointCloud(*cloud, *transformed_cloud, transform);
    std::ostringstream stream;
    //stream << std::setw(4) << std::setfill('0') << m_savedImageCnt << "_pointcloud"
    //       << ".ply";
    stream << std::setw(4) << std::setfill('0') << m_savedImageCnt << "_pointcloud" // TODO @2022/09/12 Yimin Zhao
           << ".pcd";       
    //pcl::io::savePLYFileBinary(stream.str(), *transformed_cloud);
    pcl::io::savePCDFile(stream.str(), *transformed_cloud); // TODO @2022/09/12 Yimin Zhao
    m_savedImageCnt++;
}

void Fusion::keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void*)
{
    unsigned char key = event.getKeyCode();
    if (event.keyDown())
    {
        switch (key)
        {
        case 's':
            std::cout << "Saving point cloud " << m_savedImageCnt + 1 << std::endl;
            m_saveImage = true;
            break;
        default:
            break;
        }
    }
}

/*
Initialize the PCL CloudViewer. This function only gets called once.
See http://pointclouds.org/documentation/tutorials/cloud_viewer.php for further information.
*/
void Fusion::initCloudViewer(pcl::visualization::PCLVisualizer& viewer)
{
    // Set some properties of the visualizer.
    viewer.setBackgroundColor(0.1, 0.1, 0.1);
    viewer.setShowFPS(false);
#if PCL_VERSION_COMPARE(>=, 1, 8, 0)
    viewer.removeAllCoordinateSystems();
#endif
    // Set up the visualizer's camera.
    const double pos_x = 0;
    const double pos_y = 0;
    //const double pos_z = -4000;
    const double pos_z = -200; // TODO @2022/09/12 Yimin Zhao
    const double view_x = 0;
    const double view_y = 0;
    //const double view_z = 3000;
    const double view_z = 500; // TODO @2022/09/12 Yimin Zhao
    const double up_x = 0;
    const double up_y = -1;
    //const double up_y = -0.1; // TODO @2022/09/12 Yimin Zhao
    const double up_z = 0;
    viewer.setCameraPosition(pos_x, pos_y, pos_z, view_x, view_y, view_z, up_x, up_y, up_z);

    // Set the size of the rendered points.
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3);
    viewer.registerKeyboardCallback(&Fusion::keyboardEventOccurred, *this);
}

int main(int argc, char **argv)
{
    ROS_WARN("Node is initializing..."); 
    ros::init(argc, argv, node_name.c_str()); 
    ros::NodeHandle n; 
    ros::NodeHandle pn("~");   
    ros::NodeHandle cloudNh("cloud"); 
    int exitCode = EXIT_SUCCESS;
    
    pn.param("g_frameId", g_frameId, std::string("camera_optical_frame"));
    if (!pn.getParam("frame_id", g_frameId))
    {
       ROS_WARN("~frame_id is not set! Setting frame_id to default value 'camera_optical_frame'.");
       g_frameId = "camera_optical_frame";
    }
    
    //ros::Rate loop_rate(1); 
    
    while (ros::ok())
    {
         ros::spinOnce();
         //loop_rate.sleep(); // 
         try
	     {
             // Before using any pylon methods, the pylon runtime must be initialized.
             PylonInitialize();
	         Fusion sample;
	         // Topics publish
	         sample.m_cloudPub = n.advertise<PointCloud_t>("cloud", 100);            
	         exitCode = sample.run();
	     }
	     catch (GenICam::GenericException& e)
	     {
             std::cerr << "Exception occurred: " << std::endl << e.GetDescription() << std::endl;
	         exitCode = EXIT_FAILURE;
	     }

	     // Releases all pylon resources.
	     PylonTerminate();
    }

    return exitCode;
}
