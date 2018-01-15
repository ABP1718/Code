#include "AerobicVision.h"
#include "UR_Control.h"
#include <iostream>
#include <stdio.h>
#include <string>

int main()
{	
	std::cout << "Version 1.0 team Awesome" << endl;

	unsigned int usecs = 1000000; //1sec = 1.000.000 usec
	//UR3* test = new UR3();
	Binzone* zone = new Binzone();
	Control* control = new Control(0.0001,3.0, zone);
	AerobicVision* vision = new AerobicVision("192.168.1.69");

	std::cout << "Starting in 1 seconds" << endl;
	usleep(1000000);

	cout << "Program starting" << endl;
	
	//control->printRobotLocation();

	if(!control->readCalibration()){
		cout << "No calibration file found, starting calibration.." << endl;
		control->moveToCalibratePosition();
		pcl::PointXYZ calibrationPoint = vision->Calibrate(100);
		control->Move_Calibrate(calibrationPoint);
	}
	//control->openCloseRoboticGripper(false);
	//control->openCloseRoboticGripper(true);

	pcl::visualization::CloudViewer viewer("Filtered cloud");

	bool run = true;
	while(run)
	{
		double rotation =0;

		pcl::PointCloud<pcl::PointXYZ>::Ptr showcloud;

		pcl::PointXYZ p = vision->runAlgorithm(rotation, showcloud);
		viewer.showCloud(showcloud);
		if(p.x != 1000 && p.y != 1000 && p.z != 1000)
		{
			vector<double> pos = control->getRobotLocation();
			pcl::PointXYZ rotations = control->getRxRyRz(3.1415926535897932384626433832795, 0, rotation);
			control->Touch_Object(p.z, p.y, p.x ); //, rotations.x, rotations.y, rotations.z);

			cout << "Sleeping 1 seconds" << endl;
			usleep(1000000);
			cout << "Running again.. yippie!" << endl;
		}
		else
		{
			run = false;
		}
		

		
	}

	std::cout << "Done!" << endl;
	/*
	
	bool run = true;
	int objects = 0;
	while(run){
		std::vector<objectData> data = vision_->RegionGrowing();
		objects = data.size();
		cout << "Found objects: " << objects << endl;
		if(objects == 0){ run = false;}
		else{
			//take highest (first) object and make new picture
			run = control_->Touch_Object(data[0].xyz.z, data[0].xyz.y, data[0].xyz.x,
			data[0].rx,data[0].ry,data[0].rz); 
		}
	}

	cout << "Bin Empty, Finished Bin Picking" << endl;
    	return 0;
	
	*/

	return 0;
}



