#ifndef UR_TEST_H_
#define UR_TEST_H_

#include <netdb.h>
#include <unistd.h>
#include <chrono>
#include <fcntl.h>
#include <stdio.h>
#include <stdint.h>
#include <endian.h>
#include <semaphore.h>

#include <string>
#include <queue>
#include <stdexcept>
#include <cstdarg>

#include <iostream>
#include <sstream>
#include <unistd.h>
#include <string>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>

#include "robot_state.h"
#include "robot_state_RT.h"
#include "ur_realtime_communication.h" 
#include "ur_communication.h"

//TODO: MOVE GETBINSTATE TO UR_CONTROL

class UR3{
	private:
		double firmware_version_;
		string ip_addr_;
	public:
		UR3();

		//Communication
		UrRealtimeCommunication* rt_interface_;
		UrCommunication* sec_interface_;
		
		void _send(std::string message);
		void Halt();

		//Support functions
		template <typename T> std::string to_string(T const& value);
	
		//Setters
		void stop();
		void move(double x, double y, double z, double rx, double ry, double rz);
		void moveLinear(double x, double y, double z, double rx, double ry, double rz);
		void moveHome();
		void moveGripper(bool dir);
		void setDigitalOut(unsigned int pin, bool state);
		void setAnalogOut(unsigned int pin, float value); //value range 0:1, maximum = 10V

		//Getters
		vector<double> getActualPosition();
		std::vector<bool> getDigitalInputs();		//bit 0 - 7 = Digital Input, bit 8 - 15 = Configurable Digital Input, bit 16 - 17 = Tool Digital Input	
		std::vector<bool> getDigitalOutputs();		//bit 0 - 7 = Digital Output, bit 8 - 15 = Configurable Digital Output, bit 16 - 17 = Tool Digital Output
		std::vector<double> getMotorTemperatures();
		std::vector<float> getAnalogInputs();
		std::vector<float> getAnalogOutputs();
		bool getSafety_mode();
		bool getEmergencyStop();
		bool getProtectiveStop();

		//Test software
		void move_test();
		void IO_test();
		void openCloseRoboticGripper(bool close);
		void RotateGripper90Degrees(bool clockwise);
		
};

#endif /* UR_TEST_H_ */
