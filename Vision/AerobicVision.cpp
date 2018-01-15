#include "AerobicVision.h"
#include <math.h>
//TODO: Implement Filterfunction in calibration function
//TODO: SORT GROWING ALGORITHM OUTPUT VECTOR ON Z AXIS HIGH TO LOW


AerobicVision::AerobicVision(std::string IP){
	  //logging method
	  o3d3xx::Logging::Init();
	  //initialise camera constructor expects IP address, using default one
	  cam = std::make_shared<o3d3xx::Camera>(IP);
	  //create buffer to fetch image
	  img = std::make_shared<o3d3xx::ImageBuffer>();
	  //framegrabber
	  fg = std::make_shared<o3d3xx::FrameGrabber>(cam, o3d3xx::IMG_AMP|o3d3xx::IMG_RDIS|o3d3xx::IMG_CART);
}

bool wayToSort(objectData i, objectData j){ // point x = real z
	return i.xyz.x < j.xyz.x;
}


pcl::PointCloud<pcl::PointXYZI>::Ptr AerobicVision::PassFilterPCD(pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIn, double xmin, double xmax, double ymin, double ymax, double zmin, double zmax){
	  
	pcl::PointCloud<pcl::PointXYZI>::Ptr zfilt (new pcl::PointCloud<pcl::PointXYZI>), yfilt (new pcl::PointCloud<pcl::PointXYZI>);

	//passthrough filter
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PassThrough<pcl::PointXYZI> passX, passY, passZ;

	
	passZ.setInputCloud(cloudIn);
	passZ.setFilterFieldName("z"); 		// blauw = z			REAL = X
	passZ.setFilterLimits(xmin,xmax);	
	//passZ.setFilterLimits(-0.05,0.05);	//calibration
	passZ.filter(*zfilt);
	
	passY.setInputCloud(zfilt);
	passY.setFilterFieldName("y"); 		// y= groen			REAL = Y
	passY.setFilterLimits(ymin,ymax);	//
	//passY.setFilterLimits(-0.05,0.05);	//calibratiom
	passY.filter(*yfilt);

	passX.setInputCloud(yfilt);
	passX.setFilterFieldName("x"); 		// rood = x			REAL = Z
	passX.setFilterLimits(zmin,zmax);
	//passX.setFilterLimits(0.35,0.38);	//calibration	
	passX.filter(*cloud_filtered);

	return cloud_filtered;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr AerobicVision::convertCloudXYZItoXYZ(pcl::PointCloud<pcl::PointXYZI> cloudIn){
	pcl::PCDWriter writer;
	pcl::PCDReader reader;
	writer.write<pcl::PointXYZI>("tmp_pcd.pcd", cloudIn, false);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut (new pcl::PointCloud<pcl::PointXYZ>);
	reader.read("tmp_pcd.pcd", *cloudOut);
	return cloudOut;
}

pcl::PointXYZ AerobicVision::Calibrate(int samples){
	std::vector<pcl::PointXYZ> average;
 	cout << "Start Calibration with " << samples <<  " samples" << endl;
  	//get frame from camera (could be looped to create an actual live feed)
 	for(int cnt = 0; cnt < samples; cnt++){
	  if (! fg->WaitForFrame(img.get(), 2000))
	    {
	      std::cerr << "Timeout waiting for camera!" << std::endl;
	    }
	  

	  //3D pointcloud
	  
	  //convert o3d3xx cloud to PointXYZI  
	  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIn = img->Cloud();
	  pcl::PointCloud<pcl::PointXYZI>::Ptr zfilt (new pcl::PointCloud<pcl::PointXYZI>), yfilt (new pcl::PointCloud<pcl::PointXYZI>);

	  //passthrough filter
	  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZI>);
	  pcl::PassThrough<pcl::PointXYZI> passX, passY, passZ;


	  passZ.setInputCloud(cloudIn);
	  passZ.setFilterFieldName("z"); 	// blauw = z			REAL = X
	  passZ.setFilterLimits(-0.125,0.105);	//calibration
	  passZ.filter(*zfilt);

	  passY.setInputCloud(zfilt);
	  passY.setFilterFieldName("y"); 	// y= groen			REAL = Y
	  //passY.setFilterLimits(-0.11,0.09);	//ops
	  passY.setFilterLimits(-0.1,0.075);	//calibratiom
	  passY.filter(*yfilt);

	  passX.setInputCloud(yfilt);
	  passX.setFilterFieldName("x"); 	// rood = x			REAL = Z
	  //passX.setFilterLimits(0.0,0.60);
	  passX.setFilterLimits(0.27,0.33);	//calibration
	  passX.filter(*cloud_filtered2);
	
	  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (convertCloudXYZItoXYZ(*cloud_filtered2));
	
	  pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
	  feature_extractor.setInputCloud (cloud);
	  feature_extractor.compute();

	  pcl::PointXYZ min_point_AABB;
	  pcl::PointXYZ max_point_AABB;
	  
	  feature_extractor.getAABB (min_point_AABB, max_point_AABB);
	  
	  pcl::PointXYZ middle_point;
	  middle_point.x = min_point_AABB.x + ((max_point_AABB.x - min_point_AABB.x) / 2);
	  middle_point.y = min_point_AABB.y + ((max_point_AABB.y - min_point_AABB.y) / 2);
	  middle_point.z = min_point_AABB.z + ((max_point_AABB.z - min_point_AABB.z) / 2);
	  average.push_back(middle_point);

  }
  double x = 0;
  double y = 0;
  double z = 0;
  int cnt;
  pcl::PointXYZ ret;
  for(cnt = 0; cnt < samples; cnt++){
  	x = x + average[cnt].x; 
	y = y + average[cnt].y; 
	z = z + average[cnt].z; 
  }
  //set correct XYZ format
  
  z = (z / (double)samples); 
  y = (y / (double)samples); 
  x = (x / (double)samples); 

  ret.z = x;
  ret.y = y;
  ret.x = z;
  cout << "Finished Vision Calibration" << endl;
  return ret;
}

std::vector<objectData> AerobicVision::RegionGrowing(){
	pcl::console::TicToc tt;
	// Load the input point cloud

	std::cerr << "Loading...\n", tt.tic ();
	if (! fg->WaitForFrame(img.get(), 2000))
	{
	std::cerr << "Timeout waiting for camera!" << std::endl;
	}

	pcl::PCDWriter writer;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIn = img->Cloud();
	writer.write<pcl::PointXYZI>("tmp_pcd.pcd", *cloudIn, false);	
	pcl::io::loadPCDFile ("tmp_pcd.pcd", *cloud);
	//std::cerr << ">> Done: " << tt.toc () << " ms, " << cloud->points.size () << " points\n";

	pcl::PointCloud<pcl::PointXYZ>::Ptr zfilt (new pcl::PointCloud<pcl::PointXYZ>), yfilt (new pcl::PointCloud<pcl::PointXYZ>);

	//passthrough filter
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PassThrough<pcl::PointXYZ> passX, passY, passZ;

	passZ.setInputCloud(cloud);
	passZ.setFilterFieldName("y"); 	// blauw = z			REAL = X
	passZ.setFilterLimits(-0.1,0.1);	//calibration
	passZ.filter(*zfilt);

	passY.setInputCloud(zfilt);
	passY.setFilterFieldName("z"); 	// y= groen			REAL = Y
	//passY.setFilterLimits(-0.11,0.09);	//ops
	passY.setFilterLimits(-0.12,0.12);	//calibratiom
	passY.filter(*yfilt);

	passX.setInputCloud(yfilt);
	passX.setFilterFieldName("x"); 	// rood = x			REAL = Z
	//passX.setFilterLimits(0.0,0.60);
	passX.setFilterLimits(0.0,0.6);	//calibration
	passX.filter(*cloud_filtered);

	std::vector<int> temp;
	pcl::removeNaNFromPointCloud(*cloud_filtered, *cloud_filtered, temp); 

	pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new   pcl::search::KdTree<pcl::PointXYZ>);
	pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
	normal_estimator.setSearchMethod (tree);
	normal_estimator.setInputCloud (cloud_filtered);
	normal_estimator.setKSearch (100);
	normal_estimator.compute (*normals);

	pcl::IndicesPtr indices (new std::vector <int>);
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud (cloud_filtered);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (0.0, 1.0);
	pass.filter (*indices);

	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
	reg.setMinClusterSize (50); //minimum points to find before assigning a cluster
	reg.setMaxClusterSize (1000); //maximum points to find
	reg.setSearchMethod (tree); //search using the KdTree
	reg.setNumberOfNeighbours (15); //number of neighbours??
	reg.setInputCloud (cloud_filtered); //input cloud is filtered on xyz axis
	//reg.setIndices (indices);
	reg.setInputNormals (normals); //input normals used to find the region
	reg.setSmoothnessThreshold (5.0 / 180.0 * M_PI); //how smooth has the surface to be in degrees
	reg.setCurvatureThreshold (1.0); //how much curvature can there be??

	std::vector <pcl::PointIndices> clusters;
	reg.extract (clusters);
	

	std::vector<pcl::PointCloud<pcl::PointXYZ>> clusters_with_points;
	std::vector<objectData> cluster_middle_points;
	for(int cnt = 0; cnt < clusters.size(); cnt++){
		//Create the filtering object
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		pcl::PointCloud<pcl::PointXYZ>::Ptr tmpcloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);;
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices (clusters[cnt]));
		extract.setInputCloud (cloud_filtered);
		extract.setIndices (inliers);
		extract.setNegative (false);
		extract.filter (*tmpcloud);
		clusters_with_points.push_back(*tmpcloud);

		pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
		feature_extractor.setInputCloud (tmpcloud);
		feature_extractor.compute();
	
		normal_estimator.setSearchMethod (tree);
		normal_estimator.setInputCloud (tmpcloud);
		normal_estimator.setKSearch (100);
		normal_estimator.compute (*normals);

		pcl::PointXYZ min_point_AABB;
		pcl::PointXYZ max_point_AABB;
		  
		feature_extractor.getAABB (min_point_AABB, max_point_AABB);
		 
		objectData middle_point;
		middle_point.xyz.x = min_point_AABB.x + ((max_point_AABB.x - min_point_AABB.x) / 2);
		middle_point.xyz.y = min_point_AABB.y + ((max_point_AABB.y - min_point_AABB.y) / 2);
		middle_point.xyz.z = min_point_AABB.z + ((max_point_AABB.z - min_point_AABB.z) / 2);

		//calculate single normal vector		
		float normal_x = 0.0f;
		float normal_y = 0.0f;
		float normal_z = 0.0f;
		for (int i = 0; i < normals->size();i++){
			normal_x = normal_x + normals->points[i].normal_x;
			normal_y = normal_y + normals->points[i].normal_y;
			normal_z = normal_z + normals->points[i].normal_z;
		}
		normal_x = (normal_x/normals->size());
		normal_y = (normal_y/normals->size());
		normal_z = (normal_z/normals->size());
		if(normal_x > 1 || normal_y > 1 || normal_z > 1 || normal_x < -1 || 
		normal_y < -1 || normal_z < -1) {
			cout << "Normals not correct, try again." << endl;
		} else {
			pcl::Normal normal = pcl::Normal(-normal_x,-normal_y,-normal_z);
					
			float pitch = asin(-normal_y);
			float yaw = atan2(normal_z,normal_x);
			float roll = 0.0;
			pcl::PointXYZ rpy = pcl::PointXYZ(roll,pitch,yaw);
			middle_point.pitch = pitch;
			middle_point.yaw = yaw;		
	
			
			Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitZ());
			Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitY());
			Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitX());

			Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;
			Eigen::Matrix3d rotationMatrix = q.matrix();

			double rotSum = rotationMatrix(0,0) + rotationMatrix(1,1) + rotationMatrix(2,2) - 1;
		    	double alpha = acos(rotSum / 2);
		    	double theta = 0;
		    	if (roll >= 0)
				theta = alpha;
		   	else
				theta = 2 * 3.1415926535897 - alpha;
		    	double my = 1.0 / (2 * sin(theta));
		 
		    	double rx = my * (rotationMatrix(2,1) - rotationMatrix(1,2)) * theta;
		    	double ry = my * (rotationMatrix(0,2) - rotationMatrix(2,0)) * theta;
		    	double rz = my * (rotationMatrix(1,0) - rotationMatrix(0,1)) * theta;
		 
		    	pcl::PointXYZ rotationVector = pcl::PointXYZ();
		    	rotationVector.x = (float)rx;
		    	rotationVector.y = (float)ry;
		    	rotationVector.z = (float)rz;
			middle_point.rx = rotationVector.x;
			middle_point.ry = rotationVector.y;
			middle_point.rz = rotationVector.z;
		}
	
		//add found point to vector		
		cluster_middle_points.push_back(middle_point);
	}
	
	//Sort found clusters from closest to farest	
	sort(cluster_middle_points.begin(), cluster_middle_points.end(), wayToSort);

	return cluster_middle_points;
}








///////////////////////////////////////////////////////
//Projectgroep 2 test pogingen
///////////////////////////////////////////////////////

pcl::PointCloud<pcl::PointXYZI> AerobicVision::AverageCloud(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> totalList)
{ /*
	pcl::PointCloud<pcl::PointXYZI>::Ptr toReturn = totalList[0]->makeShared();
	
	for(int i = 0; i< totalList[0]->points.size(); i++)
	{
		pcl::PointXYZI averagePoint = pcl::PointXYZI(0.0f);
		for(int k = 0; k< totalList.size();k++)
		{
			toReturn[i]->x += totalList[k]->points[i].x;
			toReturn[i]->y += totalList[k]->points[i].y;
			toReturn[i]->z += totalList[k]->points[i].z;
			toReturn[i]->intensity += totalList[k]->points[i].intensity;
		}

		toReturn[i]->x /= totalList.size();
		toReturn[i]->y /= totalList.size();
		toReturn[i]->z /= totalList.size();
		toReturn[i]->intensity /= totalList.size();
	}
	return toReturn;*/
	pcl::PointCloud<pcl::PointXYZI> toReturn;
	
	for(int i = 0; i< totalList[0]->points.size(); i++)
	{
		if(isnan(totalList[0]->points[i].x ))
		{
			toReturn.push_back(totalList[0]->points[i]);
		}
		else{

		
			pcl::PointXYZI averagePoint = pcl::PointXYZI(0.0f);
			for(int k = 0; k< totalList.size();k++)
			{
				averagePoint.x += totalList[k]->points[i].x;
				averagePoint.y += totalList[k]->points[i].y;
				averagePoint.z += totalList[k]->points[i].z;
				averagePoint.intensity += totalList[k]->points[i].intensity;
			}

			averagePoint.x /= totalList.size();
			averagePoint.y /= totalList.size();
			averagePoint.z /= totalList.size();
			averagePoint.intensity /= totalList.size();
			toReturn.push_back(averagePoint);
		}
	}
	return toReturn;
}

void AerobicVision::PrintClouds(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> totalList)
{
   for(int i = 0; i< totalList[0]->points.size(); i++)
	{
		cout << "[" << i << "] ";

		for(int k = 0; k< totalList.size();k++)
		{
			cout << std::setprecision(8) << "(" << totalList[k]->points[i].x << "," << totalList[k]->points[i].y << "," << totalList[k]->points[i].z << ")";
		}

		cout << endl;
	}
}

void AerobicVision::PrintCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
   for(int i = 0; i< cloud->points.size(); i++)
	{
		cout << "[" << i << "] ";
		cout << std::setprecision(5) << "(" << cloud->points[i].x << ", " << cloud->points[i].y << ", " << cloud->points[i].z << ")";
		cout << endl;
	}
}


void AerobicVision::Test(){
	pcl::console::TicToc tt;
	// Load the input point cloud
	std::cerr << "Loading...\n", tt.tic ();

	usleep(2000000);

	std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> allClouds;

	pcl::PointCloud<pcl::PointXYZI> cloudAverage;

	for(int i = 0; i< 4; i++)
	{

		usleep(500000);
		if (! fg->WaitForFrame(img.get(), 2000))
		{
			std::cerr << "Timeout waiting for camera!" << std::endl;
		}
		usleep(2000000);

		pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIn = img->Cloud()->makeShared();

		allClouds.push_back(cloudIn);

		std::cout << "Stored pic " << i << endl;
	}

	//cloudAverage = AverageCloud(allClouds);
	PrintClouds(allClouds);
	/*
	PrintCloud(cloudAverage);
	
	pcl::PointCloud<pcl::PointXYZI>::Ptr m_ptrCloud(&cloudAverage);

	pcl::visualization::CloudViewer viewer("Cloud Viewer", false);
    
  //blocks until the cloud is actually rendered
	viewer.showCloud(m_ptrCloud);
	*/

	cout << "Press Enter" << endl;
    cin.get();

	return;
}


void AerobicVision::AverageCloudsKdTree(pcl::PointCloud<pcl::PointXYZI>::Ptr &average, std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> allClouds)
{
	vector<pcl::KdTreeFLANN<pcl::PointXYZI>> kdtree;

	if(allClouds.size() <= 2)
		return;
	for(int g= 0; g< allClouds.size();g++)
	{
		kdtree.push_back(pcl::KdTreeFLANN<pcl::PointXYZI>());
		kdtree[g].setInputCloud(allClouds[g]);
	}
	for(int i = 0; i< allClouds[0]->points.size(); i++)
	{
		pcl::PointXYZI averagePoint = pcl::PointXYZI(0.0f);
		
		if(isnan(allClouds[0]->points[i].x))
		{
			averagePoint.x = 0;
			averagePoint.y = 0;
			averagePoint.z = 0;
			averagePoint.intensity =0;
			average->push_back(averagePoint);
			continue;
		}

		pcl::PointXYZI searchPoint;
		searchPoint.x = allClouds[0]->points[i].x;
		searchPoint.y = allClouds[0]->points[i].y;
		searchPoint.z = allClouds[0]->points[i].y;
		searchPoint.intensity = allClouds[0]->points[i].intensity;

		averagePoint.x = searchPoint.x;
		averagePoint.y = searchPoint.y;
		averagePoint.z = searchPoint.z;
		averagePoint.intensity = searchPoint.intensity;

		cout << "[" << i << "] (" << searchPoint.x << "," << searchPoint.y << "," << searchPoint.z << ") > ";

		for(int k = 1; k< allClouds.size();k++)
		{
			std::vector<int> pointIndex(1);
			std::vector<float> pointDistance(1);

			if(kdtree[k].nearestKSearch(searchPoint, 1, pointIndex, pointDistance) > 0)
			{
				cout << pointDistance[0] << ", ";
				averagePoint.x += allClouds[k]->points[pointIndex[0]].x;
				averagePoint.y += allClouds[k]->points[pointIndex[0]].y;
				averagePoint.z += allClouds[k]->points[pointIndex[0]].z;
				averagePoint.intensity += allClouds[k]->points[pointIndex[0]].intensity;
			}
			else
			{
				averagePoint.x += searchPoint.x;
				averagePoint.y += searchPoint.y;
				averagePoint.z += searchPoint.z;
				averagePoint.intensity += searchPoint.intensity;
			}
		}

		averagePoint.x /= allClouds.size();
		averagePoint.y /= allClouds.size();
		averagePoint.z /= allClouds.size();
		averagePoint.intensity /= allClouds.size();
		average->push_back(averagePoint);

		cout << " < (" << averagePoint.x << "," << averagePoint.y << "," << averagePoint.z << ")" << endl;
	}
}

void AerobicVision::Test2()
{
	cout << "Starting..." << endl;

	usleep(500000);

	std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> allClouds;

	for(int i = 0; i< 4; i++)
	{
		usleep(200000);
		if (! fg->WaitForFrame(img.get(), 2000))
		{
			std::cerr << "Timeout waiting for camera!" << std::endl;
		}
		usleep(500000);

		pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIn = img->Cloud()->makeShared();
		allClouds.push_back(cloudIn);
		std::cout << "Taken pic " << i << endl;
	}

	//New cloud
	pcl::PointCloud<pcl::PointXYZI>::Ptr average (new pcl::PointCloud<pcl::PointXYZI>);

	average->width = allClouds[0]->width;
	average->height = allClouds[0]->height;
	average->points.resize(allClouds[0]->points.size());

	//Average all clouds
	AverageCloudsKdTree(average, allClouds);

	//PrintCloud(average);

	
	//Visualize cloud
	pcl::visualization::CloudViewer viewer("Cloud Viewer");
	viewer.showCloud(average);
	

	cout << "Press Enter" << endl;
    cin.get();

	return;
}



///////////////////////////////////////////////////////
//Projectgroep 2 implementatie
///////////////////////////////////////////////////////

pcl::PointXYZ AerobicVision::runAlgorithm(double &rotation, pcl::PointCloud<pcl::PointXYZ>::Ptr &showcloud)
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
      return pcl::PointXYZ(0,0,0);	
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
  
  // cut the x axis of pcl
  pcl::PassThrough<pcl::PointXYZ> passx;
  passx.setInputCloud (vox_cloud);
  passx.setFilterFieldName ("x");
  passx.setFilterLimits (0.3, .63);//0.3,0.63
  passx.filter (*xfilt);
  /*
    pcl::visualization::CloudViewer viewer("Filtered cloud");
   viewer.showCloud(xfilt);

  while (!viewer.wasStopped ())
	{
	
	}*/

  vector<pcl::PointXYZ> points;
  if(xfilt->points.size()>0)
  {
	GetObject(xfilt, points);
  }
  else
  {
	  return pcl::PointXYZ(1000,1000,1000);
  }

  showcloud = xfilt;


  rotation = GetRotation(xfilt,points);
/*
  std::vector <pcl::PointIndices> indices;
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> filteredCloud;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr color (new pcl::PointCloud<pcl::PointXYZRGB>);
  //Eigen::Vector4f min_pt,max_pt;
  //pcl::getMinMax3D(xfilt,indices,min_pt,max_pt);
  getClusters(xfilt, indices, color);
  
  for(int i = 0; i < indices.size();i++)
  {
	  pcl::PointCloud<pcl::PointXYZ>::Ptr cld (new pcl::PointCloud<pcl::PointXYZ>);
	  filteredCloud.push_back(cld); 

	  for(int k = 0; k < indices[i].indices.size(); k++)
	  {
	  	filteredCloud[i]->push_back(xfilt->points[ indices[i].indices[k] ]);
	  }
  }

  FeatureExtractor(filteredCloud);


  /*
  for(int i = 0; i < filteredCloud.size(); i++)
  {
	cout << "Opening viewer for cloud " << to_string(i) << "/" << filteredCloud.size() << endl;

	pcl::visualization::CloudViewer viewer("Filtered cloud #" + to_string(i));
	viewer.showCloud(filteredCloud[i]);

	usleep(1000000);

	cout << "Press enter to continue" << endl;

	while (!viewer.wasStopped ()){}

	//pcl::visualization::PCLVisualizer viewer ("Filtered cloud #" + to_string(i));
	//viewer.addPointCloud<pcl::PointXYZ>(filteredCloud[i],"cloud");

	//while (!viewer.wasStopped ())
	//{
	//	viewer.spinOnce(100);
	//}
  }
  
  cin.ignore();
  */

  return points[2];
}
double AerobicVision::GetRotation(pcl::PointCloud<pcl::PointXYZ>::Ptr &in,vector<pcl::PointXYZ> &points)
{
	
	Eigen::Vector3f major,middle,minor;
	double rotation;
	pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
	feature_extractor.setInputCloud(in);
	feature_extractor.compute();
	feature_extractor.getEigenVectors (major,middle,minor);
	float length,height;
	length = points[1].y - points[0].y;
	height = points[1].z - points[0].z;
	bool useMajorVector = false;
	if(height > length)
	{
		if(height < 0.04)
		{
			useMajorVector = true;
		}
	}
	else
	{
		if(length > height)
		{
			if(length < 0.04)
			{
				useMajorVector = true;
			}
		}
	}
	if(useMajorVector)
	{
		
		rotation = acos(major[2]);
	}
	else
	{
		
		rotation = acos(middle[2]);
	}
	cout << setprecision(15) << rotation << endl;
	//cout << major[0] << "," << major[1] << "," << major[2] << endl;
	//cout << middle[0] << "," << middle[1] << "," << middle[2] << endl;
	//cout<< rotation.x <<"," << rotation.y << "," << rotation.z << endl;
	
	return rotation;
}
bool AerobicVision::GetObject(pcl::PointCloud<pcl::PointXYZ>::Ptr &in, vector<pcl::PointXYZ> &out)
{
	float treshold = 0.01f;
	pcl::PointXYZ highPoint = pcl::PointXYZ(1000,1000,1000);
	pcl::PointXYZ AA,BB;
	pcl::PointCloud<pcl::PointXYZ>::Ptr toFilter(new pcl::PointCloud<pcl::PointXYZ>);
	for(int i = 0; i< in->points.size();i++)
	{
		if(in->points[i].x < highPoint.x)
		{
			highPoint = in->points[i];
		}
	}
	  // cut the z axis of pcl

   
  // cut the x axis of pcl
  pcl::PassThrough<pcl::PointXYZ> passx;
  passx.setInputCloud (in);
  passx.setFilterFieldName ("x");
  passx.setFilterLimits (highPoint.x-treshold, highPoint.x +treshold);//0.3,0.63
  passx.filter (*toFilter);

  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr color (new pcl::PointCloud<pcl::PointXYZRGB>);
  getClusters(toFilter, clusters, color); 

  AA = clusters[0]->points[0];
  BB = clusters[0]->points[0];

/*
	   pcl::visualization::CloudViewer viewer("Filtered cloud #");
   viewer.showCloud(clusters[0]);

  while (!viewer.wasStopped ())
	{
	
	}
*/

  for(pcl::PointXYZ point : clusters[0]->points)
  {
	if(point.y < AA.y)
	{
		AA.y = point.y;
	}
	if(point.y > BB.y)
	{
		BB.y = point.y;
	}
	if(point.z < AA.z)
	{
		AA.z = point.z;
	}
	if(point.z> BB.z)
	{
		BB.z = point.z;
	}
  }

  out.push_back(AA);
  out.push_back(BB);
  highPoint.y = (AA.y+BB.y)/2;
  highPoint.z = (AA.z+BB.z)/2;
  out.push_back(highPoint);

  std::cout << "Found AA " << AA.x << ", " << AA.y << ", " << AA.z << endl;
  std::cout << "Found BB " << BB.x << ", " << BB.y << ", " << BB.z << endl;
  std::cout << "Found CC " << highPoint.x << ", " <<  highPoint.y << ", " <<  highPoint.z << endl;

  return true;
}

void AerobicVision::copyCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr &in, pcl::PointCloud<pcl::PointXYZ>::Ptr &out)
{
  for(int i = 0; i< in->points.size(); i++)
  {
    pcl::PointXYZ p = pcl::PointXYZ(in->points[i].x, in->points[i].y, in->points[i].z);
    out->push_back(p);
  }
}

void AerobicVision::getClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr &in, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &out, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &colorout)
{
  // find the normals
  pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  tree->setInputCloud(in);

  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setInputCloud (in);
  //normal_estimator.setKSearch (50);
  normal_estimator.setRadiusSearch (0.001);
  normal_estimator.compute (*normals);
    
  // region growing
  pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
  std::vector <pcl::PointIndices> indices;

  reg.setMinClusterSize (25);
  reg.setMaxClusterSize (2000);
  reg.setSearchMethod (tree);
  reg.setNumberOfNeighbours (15);
  reg.setInputCloud (in);
  //reg.setIndices (indices);
  reg.setInputNormals (normals);
  reg.setSmoothnessThreshold (2.0 / 180.0 * M_PI);// graden naar radial
  reg.setCurvatureThreshold (1.0);
  reg.extract (indices);

  colorout = reg.getColoredCloud();

  for(int i = 0; i < indices.size();i++)
  {
	  pcl::PointCloud<pcl::PointXYZ>::Ptr cld (new pcl::PointCloud<pcl::PointXYZ>);
	  out.push_back(cld); 

	  for(int k = 0; k < indices[i].indices.size(); k++)
	  {
	  	out[i]->push_back(in->points[ indices[i].indices[k] ]);
	  }
  }
}

bool AerobicVision::FeatureExtractor(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &in)
{
  for(int i = 0; i < in.size()-1; i++)
  {

	cout << "Showing object " << i << endl;

	pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
	feature_extractor.setInputCloud (in[i]);
	feature_extractor.compute ();

	std::vector <float> moment_of_inertia;
	std::vector <float> eccentricity;
	pcl::PointXYZ min_point_AABB;
	pcl::PointXYZ max_point_AABB;
	pcl::PointXYZ min_point_OBB;
	pcl::PointXYZ max_point_OBB;
	pcl::PointXYZ position_OBB;
	Eigen::Matrix3f rotational_matrix_OBB;
	float major_value, middle_value, minor_value;
	Eigen::Vector3f major_vector, middle_vector, minor_vector;
	Eigen::Vector3f mass_center;

	feature_extractor.getMomentOfInertia (moment_of_inertia);
	feature_extractor.getEccentricity (eccentricity);
	feature_extractor.getAABB (min_point_AABB, max_point_AABB);
	feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
	feature_extractor.getEigenValues (major_value, middle_value, minor_value);
	feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
	feature_extractor.getMassCenter (mass_center);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
	viewer->addCoordinateSystem (1.0);
	viewer->initCameraParameters ();
	viewer->addPointCloud<pcl::PointXYZ> (in[i], "sample cloud");
	viewer->addCube (min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0, 1.0, 0.0, "AABB");

	Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
	Eigen::Quaternionf quat (rotational_matrix_OBB);
	viewer->addCube (position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, "OBB");

	pcl::PointXYZ center (mass_center (0), mass_center (1), mass_center (2));
	pcl::PointXYZ x_axis (major_vector (0) + mass_center (0), major_vector (1) + mass_center (1), major_vector (2) + mass_center (2));
	pcl::PointXYZ y_axis (middle_vector (0) + mass_center (0), middle_vector (1) + mass_center (1), middle_vector (2) + mass_center (2));
	pcl::PointXYZ z_axis (minor_vector (0) + mass_center (0), minor_vector (1) + mass_center (1), minor_vector (2) + mass_center (2));
	viewer->addLine (center, x_axis, 1.0f, 0.0f, 0.0f, "major eigen vector");
	viewer->addLine (center, y_axis, 0.0f, 1.0f, 0.0f, "middle eigen vector");
	viewer->addLine (center, z_axis, 0.0f, 0.0f, 1.0f, "minor eigen vector");

	while(!viewer->wasStopped())
	{
		viewer->spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}
  }

  return true;
}