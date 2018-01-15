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

pcl::PointCloud<pcl::PointXYZ>::Ptr convertCloud(pcl::PointCloud<o3d3xx::PointT>::Ptr inputCloud);

int main(int argc, const char **argv)
{
  //logging method
  o3d3xx::Logging::Init();
  //initialise camera constructor expects IP address
  o3d3xx::Camera::Ptr cam = std::make_shared<o3d3xx::Camera>("192.168.1.69");
  //create buffer to fetch image
  o3d3xx::ImageBuffer::Ptr img = std::make_shared<o3d3xx::ImageBuffer>();
  //framegrabber
  o3d3xx::FrameGrabber::Ptr fg =
    std::make_shared<o3d3xx::FrameGrabber>(
      cam, o3d3xx::IMG_AMP|o3d3xx::IMG_RDIS|o3d3xx::IMG_CART);

  //get frame from camera
  if (! fg->WaitForFrame(img.get(), 2000))
    {
      std::cerr << "Timeout waiting for camera!" << std::endl;
      return -1;	
    }

  //3D pointcloud
  
  //convert o3d3xx cloud to PointXYZI  
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = img->Cloud();

  //init cloud viewer
  //pcl::visualization::CloudViewer cloudViewer("Cloud Viewer");
  //show in cloud viewer
  //cloudViewer.showCloud(cloud);
  
  
  // ----------------------------------------------------------------
  // --------Create PCL visualizer with paramaters-------------------
  // ----------------------------------------------------------------
  pcl::visualization::PCLVisualizer viewer("3D Viewer");
  viewer.setCameraPosition(0,0,0,-1,0,0);
  viewer.setBackgroundColor(0,0,0);
  //viewer.addPointCloud<pcl::PointXYZI>(cloud,"Sample cloud");
  //viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Sample cloud");
  viewer.addCoordinateSystem(1.0,0,0,0,0);
  viewer.initCameraParameters();
  // ----------------------------------------------------------------
  // -----Calculate surface normals with a search radius of 0.05-----
  // ----------------------------------------------------------------
  pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
  ne.setInputCloud (cloud);
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI> ());
  ne.setSearchMethod (tree);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
  ne.setRadiusSearch (0.05);
  ne.compute (*cloud_normals);
  cout << "Found normals: " << cloud_normals->size() << endl;
  //display normals
  //viewer.addPointCloudNormals<pcl::PointXYZI, pcl::Normal> (cloud, cloud_normals, 10, 0.05, "normals1", 0);


  //--------rough surface calculation
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ(new pcl::PointCloud<pcl::PointXYZ>);
  cloudXYZ = convertCloud(img->Cloud());
  cout << "Size: " << cloudXYZ->size() << endl;
  pcl::PointCloud<pcl::Normal>::Ptr surface_normals(new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr treeXYZ(new pcl::search::KdTree<pcl::PointXYZ> ());
  treeXYZ->setInputCloud(cloudXYZ);
  n.setInputCloud(cloudXYZ);
  n.setSearchMethod(treeXYZ);
  n.setRadiusSearch(0.05);
  n.compute(*surface_normals);
  //cloud concatenate
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_surface ( new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields(*cloudXYZ, *surface_normals, *cloud_surface);
  //search tree
  pcl::search::KdTree<pcl::PointNormal>::Ptr normaltree (new pcl::search::KdTree<pcl::PointNormal>);
  normaltree->setInputCloud(cloud_surface);
  //triangulate polygons
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  pcl::PolygonMesh triangles;
  gp3.setSearchRadius(0.5);
  gp3.setMu(2.5);
  gp3.setMaximumNearestNeighbors(100);
  gp3.setMaximumSurfaceAngle(M_PI/4);
  gp3.setMinimumAngle(M_PI/18);
  gp3.setMaximumAngle(2*M_PI/3);
  gp3.setNormalConsistency(false);
  //calculate
  gp3.setInputCloud(cloud_surface);
  gp3.setSearchMethod(normaltree);
  gp3.reconstruct(triangles);
  //display
  viewer.addPolygonMesh(triangles,"surfaces", 0);
  //wait to display images/pointcloud
  while (!viewer.wasStopped ())
  {
    viewer.spinOnce (100);
  }
  cv::waitKey(0);
  return 0;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr convertCloud(pcl::PointCloud<o3d3xx::PointT>::Ptr inputCloud){
  pcl::PointCloud<o3d3xx::PointT>::Ptr cloud = inputCloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr newcloud(new pcl::PointCloud<pcl::PointXYZ>);
  
  newcloud->width = cloud->width;
  newcloud->height = cloud->height;
  newcloud->points.resize(cloud->points.size());

  for(std::size_t i = 0; i < cloud->points.size(); i++) {
    newcloud->points[i].x = cloud->points[i].x;
    newcloud->points[i].y = cloud->points[i].y;
    newcloud->points[i].z = cloud->points[i].z;
  }

  return newcloud;
}
