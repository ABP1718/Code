#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/gp3.h>
#include "o3d3xx_camera.h"
#include "o3d3xx_framegrabber.h"
#include "o3d3xx_image.h"

int main(int argc, const char **argv)
{
  //logging method
  o3d3xx::Logging::Init();
  //initialise camera constructor expects IP address, using default one
  o3d3xx::Camera::Ptr cam = std::make_shared<o3d3xx::Camera>("192.168.1.69");
  //create buffer to fetch image
  o3d3xx::ImageBuffer::Ptr img = std::make_shared<o3d3xx::ImageBuffer>();
  //framegrabber
  o3d3xx::FrameGrabber::Ptr fg =
    std::make_shared<o3d3xx::FrameGrabber>(
      cam, o3d3xx::IMG_AMP|o3d3xx::IMG_RDIS|o3d3xx::IMG_CART);

  //get frame from camera (could be looped to create an actual live feed)
  if (! fg->WaitForFrame(img.get(), 2000))
    {
      std::cerr << "Timeout waiting for camera!" << std::endl;
      return -1;	
    }
  
  //2D images
  double min, max;
  //depth image
  cv::Mat depthImage = cv::Mat(img->DepthImage());
  cv::minMaxIdx(depthImage, &min, &max);
  cout << "Depth image" << endl; 
  cout << "Min: " << min << " Max: " << max << endl;
  cout << "Type: " << depthImage.type() << endl;
  cv::convertScaleAbs(depthImage, depthImage, 255 / max);
  cv::applyColorMap(depthImage, depthImage, cv::COLORMAP_JET);
  cv::imshow("Depth image", depthImage);
  //xyz image
  cv::Mat xyzImage = cv::Mat(img->XYZImage());
  cv::minMaxIdx(xyzImage, &min, &max);
  cout << "XYZ image" << endl; 
  cout << "Min: " << min << " Max: " << max << endl;
  cout << "Type: " << xyzImage.type() << endl;
  cv::convertScaleAbs(xyzImage, xyzImage, 255 / max);
  cv::applyColorMap(xyzImage, xyzImage, cv::COLORMAP_JET);
  cv::imshow("XYZ image", xyzImage);
  //amplitude image
  cv::imshow("Raw amplitude image", img->AmplitudeImage());
  //confidence image
  cv::imshow("Raw confidence image", img->ConfidenceImage());
  cv::waitKey(0);
  //3D pointcloud
  
  //convert o3d3xx cloud to PointXYZI  
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = img->Cloud();

  // ----------------------------------------------------------------
  // --------Create PCL visualizer with paramaters-------------------
  // ----------------------------------------------------------------
  pcl::visualization::PCLVisualizer viewer("3D Viewer");
  viewer.setCameraPosition(0,0,0,-1,0,0);
  viewer.setBackgroundColor(0,0,0);
  viewer.addPointCloud<pcl::PointXYZI>(cloud,"Sample cloud");
  //viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Sample cloud");
  viewer.addCoordinateSystem(1.0,0,0,0,0);
  viewer.initCameraParameters();
  //wait to display images/pointcloud
  while (!viewer.wasStopped ())
  {
    viewer.spinOnce (100);
  }
  cv::waitKey(0);
  return 0;
}
