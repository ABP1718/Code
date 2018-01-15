//#include "UR3_Lib.h"
#include "UR_Control.h"

int main()
{		
	unsigned int usecs = 1000000; //1sec = 1.000.000 usec
	//UR3* test = new UR3();
	Binzone* zone = new Binzone();
	Control* robot = new Control(0.0001,3.0, zone);
	cout << "Starting in 2 seconds" << endl;
	usleep(2000000);

	/*
	std::thread safety(&Control::safetyTest, robot_);
	cout << "Safety Thread Started" << endl;
	std::thread binpicking(&Control::Demo, robot_);
	cout << "Started Bin Picking Thread" << endl;
	safety.join();
	binpicking.join();*/
	
	robot->Demo();
	cout << "Done, shutting down" << endl;
    return 0;
}

//Control* robot2_ = new Control(0.0001,3.0, zone);
	/*while(true){	
		cout << "Starting in 2 seconds" << endl;
		usleep(2000000);
		
		//robot_->Demo(3.0); //ADD SLEEP!
		robot_->Demo();
		//robot_->robot_->IO_test();		
	}*/

//c++ UR3_Lib.cpp robot_state.cpp robot_state_RT.cpp ur_realtime_communication.cpp ur_communication.cpp UR_Control.cpp Main.cpp -o test -pthread -std=c++11

