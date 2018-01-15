#ifndef UR_CONTROL_H_
#define UR_CONTROL_H_

#include <stdio.h>
#include <string>
#include <fstream>
#include <iostream>

#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>

#include "UR3_Lib.h"

class Binzone{
	private:
		bool taken;
		string owner;
		int key_;
	public:
		Binzone();
		bool isTaken();
		int takeZone(string owner);
		string getOwner();
		bool releaseZone(string owner, int key);
};

class Control{
	private:
		double offset_;
		double speed_;
	public:
		Control(double offset, double speed, Binzone* zone);
		UR3* robot;
		Binzone* zone_;
		double RC0[3]; //Camera nulpunten gezien vanuit Robot
		float highestZ;
		bool safety;

		void saveCalibration();
		bool readCalibration();
	
		void shutDown();
		void Move_Basic(double x, double y, double z, bool linear = false, double rx = 0.0, double ry = 3.1415, double rz = 0.0); // 0, 180, 0 3.1415
		void setSpeed(double speed);
		double getSpeed();
		void setOffset(double offset);
		double getOffset();
		
		//Safety
		bool safetyCheck();
		bool getBinState();
		bool toolSafety();
		bool getVacuum();

		bool safetyTest();

		//"Defines" 	
		int INPUT_BIN	=	0;		//Input Bin is on Digital Input #0
		int VACUUM_SEN	=	1;
		int OUTPUT_BIN 	= 	2;	
		int TOOL_IO0 	= 	16;	
		int TOOL_IO1 	= 	17;	


		//TO BE TESTED
		
	    bool Touch_Object(double cameraX, double cameraY, double cameraZ, double rx = 0.0, double ry = 3.1415, double rz = 0.0);
		void Move_StaticZ(double x, double y);
		void Move_Box();
		void Move_Initial();
		void Move_Export();
		void moveGripper(bool dir);

		void moveToCalibratePosition();
		void Move_Calibrate(pcl::PointXYZ calibrationPoint);
		void Move_Approach(double z, double rx = 0.0, double ry = 3.1415, double rz = 0.0);	
		void Move_Position_Tool(double rx, double ry, double rz);
		void printRobotLocation();
		void RotateGripper90Degrees();
		vector<double> getRobotLocation();
		void openCloseRoboticGripper(bool close);
		pcl::PointXYZ getRxRyRz(double roll,double pitch,double yaw);
};

#endif /* UR_CONTROL_H_ */
