
#include <memory>
#include <vector>
#include <iostream>
#include <pcl/common/io.h>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/gp3.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/segment_differences.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include "o3d3xx_camera.h"
#include "o3d3xx_framegrabber.h"
#include "o3d3xx_image.h"

#include <pcl/io/pcd_io.h>

void copyCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr &in, pcl::PointCloud<pcl::PointXYZ>::Ptr &out);
void testPlanarSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr &in);
void testRegionGrowing(pcl::PointCloud<pcl::PointXYZ>::Ptr &in);
void normal_est(pcl::PointCloud<pcl::PointXYZ>::Ptr &in);
pcl::PointXYZ findHighestPoint(pcl::PointCloud<pcl::PointXYZ>::Ptr &in);
//void substract_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &in, pcl::PointCloud<pcl::PointXYZ>::Ptr &out);

int main(int argc, const char **argv)
{
  //logging method
  o3d3xx::Logging::Init();
  //initialise camera constructor expects IP address, using default one
  o3d3xx::Camera::Ptr cam = std::make_shared<o3d3xx::Camera>("192.168.1.69");
  //create buffer to fetch image
  o3d3xx::ImageBuffer::Ptr img = std::make_shared<o3d3xx::ImageBuffer>();
  //framegrabber
  o3d3xx::FrameGrabber::Ptr fg =std::make_shared<o3d3xx::FrameGrabber>(cam, o3d3xx::IMG_AMP|o3d3xx::IMG_RDIS|o3d3xx::IMG_CART);

  if (! fg->WaitForFrame(img.get(), 2000))
  {
      std::cerr << "Timeout waiting for camera!" << std::endl;
      return -1;	
  }

  //3D pointcloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr xfilt (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr yfilt (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr zfilt (new pcl::PointCloud<pcl::PointXYZ>);

  //convert o3d3xx cloud to PointXYZ  
  pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud = img->Cloud();
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr segment (new pcl::PointCloud<pcl::PointXYZ>);
  copyCloud(temp_cloud,cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr vox_cloud (new pcl::PointCloud<pcl::PointXYZ>);
 
  
  
   // cut the z axis of pcl
  pcl::PassThrough<pcl::PointXYZ> passz;
  passz.setInputCloud (cloud);
  passz.setFilterFieldName ("z");
  passz.setFilterLimits (-0.125,0.105);
  passz.filter (*zfilt);

  // cut the y axis of pcl
  pcl::PassThrough<pcl::PointXYZ> passy;
  passy.setInputCloud (zfilt);
  passy.setFilterFieldName ("y");
  passy.setFilterLimits (-0.1,0.075);
  passy.filter (*yfilt);
   
  // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (yfilt);
  sor.setMeanK (100);
  sor.setStddevMulThresh (0.5);
  sor.filter (*cloud_filtered);

   // Create the filtering object
  pcl::VoxelGrid<pcl::PointXYZ> vox;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  vox.setInputCloud (cloud_filtered);
  vox.setLeafSize (0.001f, 0.001f, 0.001f);
  vox.filter (*vox_cloud);
  
  //pcl::PCDWriter writer;
	pcl::PCDReader reader;
	//writer.write<pcl::PointXYZ>("bottom.pcd", *vox_cloud, false); // 1x 

  pcl::PointCloud<pcl::PointXYZ>::Ptr read_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	reader.read("bottom.pcd", *read_cloud);

  pcl::PointXYZ point = findHighestPoint(read_cloud);
  cout <<"point x: "<< point.x <<" point y: " <<point.y <<" point z: " <<point.z << endl;
  // cut the x axis of pcl
  pcl::PassThrough<pcl::PointXYZ> passx;
  passx.setInputCloud (vox_cloud);
  passx.setFilterFieldName ("x");
  passx.setFilterLimits (0.3, .63);//0.3,0.63
  passx.filter (*xfilt);

  normal_est(xfilt);
  //testPlanarSegmentation(vox_cloud);
  //substract_cloud(vox_cloud,read_cloud);

  //pcl::visualization::CloudViewer viewer("Cloud Viewer");
  //viewer.showCloud(xfilt);

  cv::waitKey(0);
  return 0;
}

void copyCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr &in, pcl::PointCloud<pcl::PointXYZ>::Ptr &out)
{
  for(int i = 0; i< in->points.size(); i++)
  {
    pcl::PointXYZ p = pcl::PointXYZ(in->points[i].x, in->points[i].y, in->points[i].z);
    out->push_back(p);
  }
}

pcl::PointXYZ findHighestPoint(pcl::PointCloud<pcl::PointXYZ>::Ptr &in)
{
  pcl::PointXYZ fp;

  for(pcl::PointXYZ p : in->points)
  {
    if(p.x > fp.x)
      fp = pcl::PointXYZ(p.x, p.y, p.z);
  }

  return fp;
}

//void substract_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &in, pcl::PointCloud<pcl::PointXYZ>::Ptr &out)
//{
//  return 0;
//}
void testPlanarSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr &in)
{ 
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

  // PLANAR SEGMENTATION
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());

  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.001);

  /*
  seg.setInputCloud (in);
  seg.segment (*inliers, *coefficients);
  cout << inliers << " - " << coefficients <<endl;
  */
  int i=0, nr_points = (int) in->points.size ();
  while (in->points.size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (in);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (in);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);
    std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

    // Remove the planar inliers, extract the rest
    extract.setNegative (false);
    extract.filter (*cloud_f);
    *in = *cloud_f;
  }

  *cloud_plane = *in;

  // Creating the KdTree object for the search method of the extraction
  cout << " Clustersize: " << cloud_plane->points.size() << endl;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr boom (new pcl::search::KdTree<pcl::PointXYZ>);
  //pcl::search::Search<pcl::PointXYZ>::Ptr boom = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
  boom->setInputCloud(cloud_plane);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.2); // 1cm
  ec.setMinClusterSize (10);
  ec.setMaxClusterSize (1000);
  ec.setSearchMethod (boom);
  ec.setInputCloud (cloud_plane);
  ec.extract (cluster_indices);
  
  /*
  //SUBSTRACT PCL METHOD
  pcl::PointIndices::Ptr fInliers (new pcl::PointIndices);
 
  //Extract fInliers from the input cloud
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(cloud_plane);
  extract.setIndices (fInliers);
  //extract.setNegative (false); //Removes part_of_cloud but retain the original full_cloud
  extract.setNegative (false); // Removes part_of_cloud from full cloud  and keep the rest
  extract.filter (*in);
  */

  //viewer.addPointCloud<pcl::PointXYZ>(in,"cloud");
  /*
  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
  }*/

  /*
  
  

  //pcl::visualization::PCLVisualizer viewer ("Cluster viewer");

  while (!viewer.wasStopped ())
  {
      viewer.spinOnce(100);
  }
  */
  cv::waitKey(0);
  return;
}

void normal_est(pcl::PointCloud<pcl::PointXYZ>::Ptr &in)
{
  // find the normals
  pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  tree->setInputCloud(in);

  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setInputCloud (in);
  //normal_estimator.setKSearch (50);
  normal_estimator.setRadiusSearch (0.02);
  normal_estimator.compute (*normals);
    
  // region growing
  pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
  std::vector <pcl::PointIndices> clusters;

  reg.setMinClusterSize (10);
  reg.setMaxClusterSize (1000);
  reg.setSearchMethod (tree);
  reg.setNumberOfNeighbours (50);
  reg.setInputCloud (in);
  //reg.setIndices (indices);
  reg.setInputNormals (normals);
  reg.setSmoothnessThreshold (2.0 / 180.0 * M_PI);// graden naar radial
  reg.setCurvatureThreshold (5.0);
  reg.extract (clusters);

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
  pcl::visualization::PCLVisualizer viewer ("Cloud viewer");
  viewer.addPointCloud<pcl::PointXYZRGB>(colored_cloud,"cloud");
  viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(in, normals, 100, 0.01, "normals",0);

  while (!viewer.wasStopped ())
  {
      viewer.spinOnce(100);
  }


}

void testRegionGrowing(pcl::PointCloud<pcl::PointXYZ>::Ptr &in)
{
   //pcl::visualization::CloudViewer cviewer("Cloud Viewer");
  //cviewer.showCloud(vox_cloud);

  /*
  // find the normals
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setInputCloud (vox_cloud);
  normal_estimator.setKSearch (50);
  normal_estimator.compute (*normals);
    
  // region growing
  pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
  std::vector <pcl::PointIndices> clusters;

  reg.setMinClusterSize (10);
  reg.setMaxClusterSize (1000);
  reg.setSearchMethod (tree);
  reg.setNumberOfNeighbours (50);
  reg.setInputCloud (vox_cloud);
  //reg.setIndices (indices);
  reg.setInputNormals (normals);
  reg.setSmoothnessThreshold (2.0 / 180.0 * M_PI);// graden naar radial
  reg.setCurvatureThreshold (5.0);
  reg.extract (clusters);


  */
  //pcl::visualization::CloudViewer viewer("Cloud Viewer");
  //viewer.showCloud(cloud_filtered);
  
  //pcl::PointCloud <pcl::PointXYZRGB>::Ptr vox_cloud = seg.getInputNormals ();
  //pcl::PointCloud <pcl::PointXYZRGB>::Ptr vox_cloud = ec.getInputCloud();
  pcl::visualization::PCLVisualizer viewer ("Cluster viewer");
  //viewer.addPointCloud<pcl::PointXYZ>(vox_cloud,"cloud");
  //viewer.addPointCloud<pcl::PointXYZRGB>(segview,"cloud");
  
  //viewer.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(colored_cloud, normals, 10, 0.05, "normals",0);

  while (!viewer.wasStopped ())
  {
      viewer.spinOnce(100);
  }
}