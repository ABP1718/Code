#include "UR_Control.h"

//TODO: INSERT TIMEOUT TO MOVE_BASIC

Control::Control(double offset, double speed, Binzone* zone){
	robot = new UR3();
	setOffset(offset);
	setSpeed(speed);
	zone_ = zone; //Zone is niet meer nodig
	//RC0 = {0.0,0.0,0.0};
	usleep(1000000);
	//Move_Calibrate();
	//readCalibration();	
	usleep(1000000);
	highestZ = 0;
	safety = true;
	cout << "Initialized Robot" << endl;
}

void Control::setSpeed(double speed){
	speed_ = speed;
}
		
double Control::getSpeed(){
	return speed_;
}

void Control::setOffset(double offset){
	offset_ = offset;
}

double Control::getOffset(){
	return offset_;
}

bool Control::getBinState(){
	vector<bool> inputs = robot->getDigitalInputs();
	if(!inputs[INPUT_BIN]){
		cout << "Bin not placed" << endl;
	}
	return inputs[INPUT_BIN];
}

bool Control::toolSafety(){
	bool ret = true;
	vector<bool> data = robot->getDigitalInputs();
	if(data[TOOL_IO0] || data[TOOL_IO1]){
		ret = false;	
		cout << "Tool Safety Ring activated" << endl;
	}
	return ret;
}

bool Control::safetyCheck(){

	if(!robot->getSafety_mode() || !getBinState() || robot->getEmergencyStop() || robot->getProtectiveStop() || !toolSafety()){
		return false;
	}else{
		return true;	
	}
}

bool Control::safetyTest(){	
	while(safetyCheck()){
	}
	return false;
}

void Control::shutDown(){
	robot->Halt();
}

void Control::moveGripper(bool dir)
{
	robot->moveGripper(dir);
}

void Control::Move_Basic(double x, double y, double z, bool linear, double rx, double ry, double rz){

/*
	if(rx == -1 || ry == -1 || rz == -1)
	{
		vector<double> pos = getRobotLocation();
		rx = pos[3];
		ry = pos[4];
		rz = pos[5];
	}
*/
	if(safety){	
		if(!linear){
			robot->move(x,y,z,rx,ry,rz);
		}else{
			robot->moveLinear(x,y,z,rx,ry,rz);
		}

		vector<double> posData = robot->getActualPosition();
		double offset = getOffset();

		while((posData[0] > x + offset) || (posData[0] < x - offset) 	|| (posData[1] > y + offset) || (posData[1] < y - offset) 	
			|| (posData[2] > z + offset) || (posData[2] < z - offset) ||	(posData[3] > rx + 0.001) || (posData[3] < rx - 0.005) 	
			|| (posData[4] > ry + 0.005) || (posData[4] < ry - 0.005) 	|| (posData[5] > rz + 0.005) || (posData[5] < rz - 0.005)){
			posData = robot->getActualPosition();
			if(!safetyCheck()){
				safety = false;
				robot->stop();
				break;
			}
		}	
	}	
}

void Control::Move_StaticZ(double x, double y){
	double z;
	z = 0.350;
	vector<double> posData = robot->getActualPosition();
	Move_Basic(posData[0], posData[1], z);				// misschien Move_StaticXY van maken
	Move_Basic(x, y, z);
}

void Control::Move_Initial(){
	double x, y;
	x = 0.10500;
	y = -0.37900;
	Move_StaticZ(x,y);
}

void Control::Move_Box(){
	double x, y;
	x = -0.31400;
	y = -0.12330;
	Move_StaticZ(x,y);
}

void Control::Move_Export(){
	double x, y;
	x = 0.34905;
	y = -0.01155;
	Move_StaticZ(x,y);
}

void Control::Move_Approach(double z, double rx, double ry, double rz){
	vector<double> posData  = robot->getActualPosition();
	Move_Basic(posData[0], posData[1], z,true, rx, ry, rz);				// misschien Move_StaticXY van maken
}

void Control::moveToCalibratePosition(){
	Move_Initial();
	Move_Box();
	double rx = 0.000;
	double ry = 0.000;
	double rz = 0.000;
	double x = -0.28200; 	//-304mm
	double y = -0.13900; 	//-135mm
	double z = 0.11000;	//210mm
	Move_Basic(x, y, z, false);
	usleep(1000000);
	Move_Basic(x,y,0.33000,false,rx,ry,rz);
	cout << "Place Calibration Head and Press Enter" << endl;	
	cin.get();
	usleep(1000000);
}

void Control::Move_Calibrate(pcl::PointXYZ calibrationPoint){
	double rx = 0.000;
	double ry = 0.000;
	double rz = 0.000;
	double x = -0.28200; 	//-304mm
	double y = -0.13900; 	//-135mm
	double z = 0.11000;	//210mm
	
	if(safety){
		vector<double> posData  = robot->getActualPosition();
		cout << "calibrationPoint(XYZ): " << calibrationPoint.x << " , " << calibrationPoint.y << " , " << calibrationPoint.z << endl;

		cout << "RobotPosition(XYZ): " << posData[0] << " , " << posData[1] << " , " << posData[2] << endl;

		RC0[0] = posData[0] - (calibrationPoint.x * -1);	//x
	 	RC0[1] = posData[1] - (calibrationPoint.y * -1);	//y
		RC0[2] = posData[2] - (calibrationPoint.z * -1);	//z
	
		saveCalibration();
		usleep(1000000);
		cout << "Place Gripper Head and Press Enter" << endl;
		cin.get();
		usleep(1000000);
		Move_Basic(-0.511,-0.169,0.275, false, 1.172, -1.222,-1.045);
		Move_Basic(x, y, z, false);	
		Move_Box();
		Move_Initial();	
	}
}

void Control::saveCalibration(){
	cout << "Saving Calibration..." << endl;
	cv::Mat coordinates = (cv::Mat_<double>(3,1) << RC0[0] , RC0[1], RC0[2]);
	cv::FileStorage fs("calibration.yml", cv::FileStorage::WRITE);
	fs << "CalibrationCoordinates" << coordinates;
	fs.release();
	cout << "Calibration Saved!" << endl;
}
bool Control::readCalibration(){
	cout << "Reading Calibration... " << endl;
	cv::FileStorage fs("calibration.yml", cv::FileStorage::READ);
	if(!fs.isOpened()){ return false; }
	cv::Mat coordinates;
	fs["CalibrationCoordinates"] >> coordinates;
	cout << "x: " << coordinates.at<double>(0,0)<< endl;
	cout << "y: " << coordinates.at<double>(0,1)<< endl;
	cout << "z: " << coordinates.at<double>(0,2)<< endl;
	RC0[0] = coordinates.at<double>(0,0);
	RC0[1] = coordinates.at<double>(0,1);
	RC0[2] = coordinates.at<double>(0,2);
	cout << "Calibration Loaded!"<< endl;
	return true;
}

bool Control::Touch_Object(double cameraX, double cameraY, double cameraZ, double rx, double ry, double rz){ //BEFORE IMPLEMENTATION Z: implement offset
	if(safety){
		
		Move_Initial();
		Move_Box();
		//openCloseRoboticGripper(false);
		double z_offset = 0.005; //0.005 == 5mm
		
		double x = RC0[0] + (cameraX * -1);
		double y = RC0[1] + (cameraY * -1);
		double z = RC0[2] - z_offset + (cameraZ * -1) + 0.145; //0.145 voor normalen  en 0.225 voor de verlengde stukken
		highestZ = z + 0.05;
		Move_StaticZ(x,y);
		//cout << "open Gripper";
		//usleep(2500000);
		Move_Approach(highestZ, rx, ry, rz);
		//robot->setDigitalOut(0, false);
		Move_Approach(z, rx, ry, rz);

	//	cout << "close Gripper";

		//robot->setDigitalOut(0, false);
		usleep(2500000);
		Move_Approach(highestZ, rx, ry, rz);
		Move_Approach(0.32);
		Move_Box();
		Move_Initial();
		Move_Export();
		//cout << "open Gripper";
		usleep(500000);
		//moveGripper(true);
		//usleep(2500000);

		/*
		if(getVacuum()){
			Move_Export();
			Move_Approach(0.25);
			robot->setDigitalOut(0, true);
			Move_Approach(0.30);
			Move_Initial();
		}else{
			robot->setDigitalOut(0, true);
		}
		*/
		return true;
	}else{
		return false;
	}
}

void Control::Move_Position_Tool(double rx, double ry, double rz){
	vector<double> posData = robot->getActualPosition();
	Move_Basic(posData[0], posData[1], posData[2], false, rx, ry, rz);

}

bool Control::getVacuum(){
	vector<bool> inputs = robot->getDigitalInputs();
	if(!inputs[VACUUM_SEN]){
		cout << "Missing Object" << endl;
	}
	return inputs[VACUUM_SEN];	
}


Binzone::Binzone(){
	taken = false;
	owner = "none";
	key_ = 0;
}

bool Binzone::isTaken(){
	return taken;
}

int Binzone::takeZone(string owner){
	int ret;
	if(!isTaken()){		//Zone is free
		ret = rand();
		owner = owner;	
		taken = true;
		key_ = ret;
		cout << "Zone is taken by " << owner << ". Generated Key: " << ret << endl;
	}else{			//Zone is taken
		cout << "Zone is already taken by " << getOwner() << endl;
		ret = 0;
	}
	return ret;
}

string Binzone::getOwner(){
	return owner;
}

bool Binzone::releaseZone(string owner, int key){
	bool ret;	
	if(isTaken()){	
		cout << "Original Key: " << key_ << " - Received Key: " << key << endl;
		if(key == key_){
			cout << "Zone released" << endl;
			owner = "none";
			key_ = 0;
			taken = false;
			ret = true;
		}else{
			cout << "Key not correct" << endl;
			ret = false;
		}	
	}else{
		cout << "Zone is free" << endl;
		ret = false;
	}
	return ret;
}

void Control::printRobotLocation()
{
	vector<double> posData = robot->getActualPosition();
	for(int i = 0; i < posData.size();i++)
	{
		cout<< posData[i] <<"  :" << i<<" ";
	}
}

vector<double> Control::getRobotLocation()
{
	vector<double> posData = robot->getActualPosition();
	return posData;
}

void Control::openCloseRoboticGripper(bool close)
{
	robot->openCloseRoboticGripper(close);
}

void Control::RotateGripper90Degrees(){
	
}
pcl::PointXYZ Control::getRxRyRz(double rollval,double pitchval,double yawval)
{
	double yaw[3][3] =  {
						{cos(yawval), -sin(yawval), 0.0},
						{sin(yawval), cos(yawval), 0.0},
						{0.0, 0.0, 1.0}
						};
	
	double pitch[3][3] = {
						 {cos(pitchval), 0.0, sin(pitchval)},
						 {0.0, 1.0, 0.0},
						 {-sin(pitchval), 0.0, cos(pitchval)}
						 };
	
	double roll[3][3] = {
						{1.0, 0.0, 0.0},
						{0.0, cos(rollval), -sin(rollval)},
						{0.0, sin(rollval), cos(rollval)}
						};
	
	double R[3][3];
	double temp[3][3];

	int i = 0;
	int j = 0;
	int k = 0;

	for(i = 0; i < 3; ++i)
	{
        for(j = 0; j < 3; ++j)
        {
			temp[i][j]=0.0;
			R[i][j]=0.0;
        }
	}

	for(i = 0; i < 3; ++i)
	{
        for(j = 0; j < 3; ++j)
		{
            for(k = 0; k < 3; ++k)
            {
                temp[i][j] += yaw[i][k] * pitch[k][j];
            }
		}
	}

	for(i = 0; i < 3; ++i)
	{
        for(j = 0; j < 3; ++j)
		{
				cout << "T " << i << "," << j << " is " << temp[i][j] << endl;
		}
	}

	for(i = 0; i < 3; ++i)
	{
        for(j = 0; j < 3; ++j)
		{
            for(k = 0; k < 3; ++k)
            {
                R[i][j] += temp[i][k] * roll[k][j];

            }
		}
	}

	for(i = 0; i < 3; ++i)
	{
        for(j = 0; j < 3; ++j)
		{
				cout << "R " << i << "," << j << " is " << R[i][j] << endl;
		}
	}


	double theta = acos(((R[0][0] + R[1][1] + R[2][2]) - 1.0) / 2.0);

	cout << "Theta " << theta << endl;


	double multi = 1.0 / (2.0 * sin(theta));

	cout << "Multi " << multi << endl;

	double rx = multi * (R[2][1] - R[1][2]) * theta;
	double ry = multi * (R[0][2] - R[2][0]) * theta;
	double rz = multi * (R[1][0] - R[0][1]) * theta;

	cout << rx << "," << ry << "," << rz << endl;

	pcl::PointXYZ p(rx, ry, rz);

	return p;
}