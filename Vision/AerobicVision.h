#include <iostream>
#include <algorithm>
#include <memory>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/gp3.h>
#include "o3d3xx_camera.h"
#include "o3d3xx_framegrabber.h"
#include "o3d3xx_image.h"
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <vector>
#include <pcl/filters/extract_indices.h>
#include <math.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/region_growing.h>
#include <boost/thread/thread.hpp>
#include <pcl/console/time.h>
#include <stdio.h>
#include <string>

using namespace std;

typedef pcl::PointXYZI PointTypeIO;
typedef pcl::PointXYZINormal PointTypeFull;

struct objectData {
	pcl::PointXYZ xyz;
	float pitch;
	float yaw;
	float rx, ry, rz;		
};

class AerobicVision{
	private:
		o3d3xx::Camera::Ptr cam;
		o3d3xx::ImageBuffer::Ptr img;
		o3d3xx::FrameGrabber::Ptr fg;
	public:
		AerobicVision(std::string IP);
		pcl::PointCloud<pcl::PointXYZI>::Ptr PassFilterPCD(pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIn, double xmin, double xmax, double ymin, double ymax, double zmin, double zmax);
		pcl::PointCloud<pcl::PointXYZ>::Ptr convertCloudXYZItoXYZ(pcl::PointCloud<pcl::PointXYZI> cloudIn);		
		pcl::PointXYZ Calibrate(int samples);
 		std::vector<objectData> RegionGrowing();
		pcl::PointXYZ ToRotVector(pcl::PointXYZ rpy);
		//bool wayToSort(pcl::PointXYZ i, pcl::PointXYZ j);
		pcl::PointCloud<pcl::PointXYZI> AverageCloud(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> totalList);
		void PrintClouds(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> totalList);
		void PrintCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
		void Test();
		void AverageCloudsKdTree(pcl::PointCloud<pcl::PointXYZI>::Ptr &average, std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> allClouds);
		void Test2();
		pcl::PointXYZ runAlgorithm(double&, pcl::PointCloud<pcl::PointXYZ>::Ptr &showcloud);
		void copyCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr &in, pcl::PointCloud<pcl::PointXYZ>::Ptr &out);
		void getClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr &in, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &out, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &colorout);
		bool GetObject(pcl::PointCloud<pcl::PointXYZ>::Ptr &in, vector<pcl::PointXYZ> &out);
		bool FeatureExtractor(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &in);
		double GetRotation(pcl::PointCloud<pcl::PointXYZ>::Ptr &in,vector<pcl::PointXYZ> &points);
};
