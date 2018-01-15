//#include <boost/asio.hpp>
//#include <boost/bind.hpp>
//#include <boost/thread.hpp>
//#include <boost/signals2.hpp>
//#include <boost/array.hpp>
//#include <boost/lexical_cast.hpp>
#include "UR3_Lib.h"

using namespace std;

//TODO: COPY UR_DRIVER FUNCTIONS
//TODO: IMPLEMENT DELAY FOR ALL GET FUNCTIONS BECAUSE OF REFRESH RATE DATA (+/- 128Hz)


template <typename T> std::string UR3::to_string(T const& value){
	stringstream sstr;
	sstr << value;
	return sstr.str();
}

UR3::UR3(){
	condition_variable msg_cond;
	unsigned int safety_count_max = 12;
	rt_interface_ = new UrRealtimeCommunication(msg_cond, "192.168.1.11", safety_count_max);
	sec_interface_ = new UrCommunication(msg_cond, "192.168.1.11");
	if(!sec_interface_->start()){
		cout << "Communication not started" << endl;
	}
	firmware_version_ = sec_interface_->robot_state_->getVersion();
	cout << "Firmware Version: " << firmware_version_ << endl;	
	rt_interface_->robot_state_->setVersion(firmware_version_);

	if(!rt_interface_->start()){
		cout << "RT Communication not started" << endl;
	}else{
		cout << "RT Communication started" << endl;
	}

}

void UR3::_send(std::string message){
	rt_interface_->addCommandToQueue(message);
}

void UR3::move(double x, double y, double z, double rx, double ry, double rz){
	string xco, yco, zco, rxo, ryo,rzo, message;
	xco = to_string(x);
	yco = to_string(y);
	zco = to_string(z);
	rxo = to_string(rx);
	ryo = to_string(ry);
	rzo = to_string(rz);
	
	message = "movej(p[" +xco + ", " + yco + ", " + zco + ", " +rxo+", "+ryo+", "+rzo+"], a=1.0,v=1.0)\nend\n"; //a=1.0,v=1.0
	//message = "movej(p[" +xco + ", " + yco + ", " + zco + "], a=1.0,v=1.0)\nend\n";
	_send(message);
}

void UR3::moveLinear(double x, double y, double z, double rx, double ry, double rz){
	string xco, yco, zco, rxo, ryo,rzo, message;
	xco = to_string(x);
	yco = to_string(y);
	zco = to_string(z);
	rxo = to_string(rx);
	ryo = to_string(ry);
	rzo = to_string(rz);
	
	message = "movel(p[" +xco + ", " + yco + ", " + zco + ", " +rxo+", "+ryo+", "+rzo+"], a=0.2,v=0.2)\nend\n";
	_send(message);
}

void UR3::moveHome(){
	string message = "movep(p[0.0,-getActualPosition90.0,0.0,-90.0,0.0,0.0], a=0.2,v=0.2)\nend\n"; //a=0.2,v=0.2
	_send(message);
}

void UR3::moveGripper(bool close){
	string message;
	if(close)
		message = "speedj([0.0,0.0,0.0,0.0,0.0,2.5], 1.0, 3.3)\nend\n"; //a=0.2,v=0.2
	else{
		message = "speedj([0.0,0.0,0.0,0.0,0.0,-2.5], 1.0, 3.3)\nend\n"; //a=0.2,v=0.2
	}
	_send(message);
	usleep(3100000);
	_send("stopj(1.0)\nend\n");
}
void UR3::openCloseRoboticGripper(bool close){
	/*
	string message = "rq_reset()\nrq_activate()\n";
	//_send("rq_reset()\nend\n");
	//usleep(500000);
	//_send("rq_activate()\nend\n");
	//usleep(500000);
	if(close)
		message += "rq_move(0)\nend\n"; 
	else{
		message += "rq_move(255)\nend\n"; 
		//message = "rqc_open()\nend\n"; 
	}
	cout << message;
	_send(message);
	usleep(1000000);
	*/

	/*
	string message = "socket_close(\"gripper_socket\")\nsync()\nsocket_open(\"127.0.0.1\", 63352, \"gripper_socket\")\nsync()\n";
	message += "socket_set_var(\"GTO\", 1, \"gripper_socket\")\n";
	message += "socket_set_var(\"FOR\", 255, \"gripper_socket\")\n";
	message += "socket_set_var(\"SPE\", 150, \"gripper_socket\")\n";

	if(close)
		message += "socket_set_var(\"POS\", 0, \"gripper_socket\")\n";
	else{
		message += "socket_set_var(\"POS\", 255, \"gripper_socket\")\n";
		//message = "rqc_open()\nend\n"; 
	}

	message += "end\n";

	_send(message);
	*/

	_send("socket_close(\"gripper_socket\")\n");
	usleep(100000);
	_send("socket_open(\"127.0.0.1\", 63352, \"gripper_socket\")");
	usleep(100000);
	_send("socket_set_var(\"GTO\", 1, \"gripper_socket\")\n");
	usleep(100000);
	_send("sleep(1)\n");
	usleep(1000000);
	_send("socket_set_var(\"FOR\", 255, \"gripper_socket\")\n");
	usleep(100000);
	_send("socket_set_var(\"SPE\", 150, \"gripper_socket\")\n");
	usleep(100000);

	if(close)
		_send("socket_set_var(\"POS\", 0, \"gripper_socket\")\n");
	else{
		_send("socket_set_var(\"POS\", 255, \"gripper_socket\")\n");
		//message = "rqc_open()\nend\n"; 
	}
	usleep(100000);
	_send("end\n");

	usleep(3000000);
}
std::vector<bool> UR3::getDigitalInputs(){
	usleep(1000);
	//vector<bool> data = rt_interface_->robot_state_->getDigitalInputBits(); //RT dig in function - High speed
	unsigned int data = sec_interface_->robot_state_->getDigitalInputBits();
	unsigned int check = 1;
	vector<bool> ret;
	for(int i = 0; i < 18; i++){
		if((data & check) == 1){ ret.push_back(true);}
		else{ret.push_back(false);}
		data = data >> 1;
	}
	return ret;
}

std::vector<bool> UR3::getDigitalOutputs(){
	usleep(1000);
	unsigned int data = sec_interface_->robot_state_->getDigitalOutputBits();
	//cout << "Digital Outputs: " << data << endl;
	unsigned int check = 1;
	vector<bool> ret;
	for(int i = 0; i < 18; i++){
		if((data & check) == 1){ ret.push_back(true);}
		else{ret.push_back(false);}
		data = data >> 1;
	}
	return ret;
}

void UR3::setDigitalOut(unsigned int pin, bool state){
	string p,s,message; 
	p = to_string(pin);
	s = to_string(state ? "True" : "False");
	message = "sec setOut():\n\tset_standard_digital_out("+p+","+s+")\nend\n";
	_send(message);
}

std::vector<float> UR3::getAnalogInputs(){
	vector<float> ret;
	ret.push_back(sec_interface_->robot_state_->getAnalogInput0());
	ret.push_back(sec_interface_->robot_state_->getAnalogInput1());
	return ret;
}

std::vector<float> UR3::getAnalogOutputs(){
	vector<float> ret;
	ret.push_back(sec_interface_->robot_state_->getAnalogOutput0());
	ret.push_back(sec_interface_->robot_state_->getAnalogOutput1());
	return ret;
}

void UR3::setAnalogOut(unsigned int pin, float value){
	string p,v,message;
	p = to_string(pin);
	v = to_string(value);
	message = "sec setOut():\n\tset_analog_out("+p+","+v+")\nend\n"; 
	_send(message);
}

vector<double> UR3::getActualPosition(){
	vector<double> ret = rt_interface_->robot_state_->getToolVectorActual();
	return ret;
}

bool UR3::getSafety_mode(){
	double ret = rt_interface_->robot_state_->getSafety_mode();
	if(ret == 3.0){
		cout << "Safety: Safety Stop" << endl;
		return false;	
	}else if(ret == 2.0){
		cout << "Safety: Limited Operational" << endl;
		return true;
	}else{
		return true;
	}
}

void UR3::IO_test(){
	vector<bool> dig_in = getDigitalInputs();
	for(int i = 0; i < dig_in.size(); i++){
		cout << "Digital Input Pin " << i << " - State: " << dig_in[i] << endl;
	}
	vector<bool> dig_out = getDigitalOutputs();
	for(int i = 0; i < dig_out.size(); i++){
		cout << "Digital Output Pin " << i << " - State: " << dig_out[i] << endl;
	}
	vector<float> analog_in = getAnalogInputs();
	for(int i = 0; i < analog_in.size(); i++){
		cout << "Analog Input Pin " << i << " - Value: " << analog_in[i] << endl;
	}
	vector<float> analog_out = getAnalogOutputs();
	for(int i = 0; i < analog_out.size(); i++){
		cout << "Analog Output Pin " << i << " - Value: " << analog_out[i] << endl;
	}
}

void UR3::move_test(){
		double rx, ry, rz;
		rx = 0.0395;
		ry = 3.1199;
		rz = -0.0102;

		setDigitalOut(0, true);
		
		move(0.43269,-0.05873,0.12940,rx,ry,rz);	//move above object at A	
		move(0.43269,-0.05873,-0.08640,rx,ry,rz);	//touch object at A
		setDigitalOut(0, false);			//vacuum at A
		move(0.43269,-0.05873,0.12940,rx,ry,rz);	//remove object at A
		//move(-0.04905,-0.33974,0.72268,0.4840,0.5834,-1.5299,time);
		move(-0.30924,-0.26365,0.12940,rx,ry,rz);	//move object A to B
		move(-0.30924,-0.26365,-0.08640,rx,ry,rz);	//touch object at B
		setDigitalOut(0, true);				//release vacuum at B
		move(-0.30924,-0.26365,0.12940,rx,ry,rz);	//remove object at B
		move(-0.30924,-0.26365,-0.08640,rx,ry,rz);	//touch object at B
		setDigitalOut(0, false);			//vacuum at B
		move(-0.30924,-0.26365,0.12940,rx,ry,rz);	//remove object at B
		//move(-0.04905,-0.33974,0.72268,0.4840,0.5834,-1.5299,time);
		move(0.43269,-0.05873,0.12940,rx,ry,rz);	//move object B to A
		move(0.43269,-0.05873,-0.08640,rx,ry,rz);	//touch object at A
		setDigitalOut(0, true);				//vacuum at A
		move(0.43269,-0.05873,0.12940,rx,ry,rz);	//remove object at A

}

void UR3::Halt(){
	rt_interface_->halt();
}

void UR3::stop(){
	string message = "sec setOut():\n\tstop()\nend\n";
	_send(message);
}

bool UR3::getEmergencyStop(){
	bool ret = sec_interface_->robot_state_->isEmergencyStopped();
	if(ret){cout << "Emergency Stop" << endl;}
	return ret;
}

bool UR3::getProtectiveStop(){
	bool ret = sec_interface_->robot_state_->isEmergencyStopped();
	if(ret){cout << "Protective Stop" << endl;}
	return sec_interface_->robot_state_->isProtectiveStopped();
}

std::vector<double> UR3::getMotorTemperatures(){
	return rt_interface_->robot_state_->getMotorTemperatures();
}


