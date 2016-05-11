// Standard Libraries
#include <iostream>
#include <math.h>
#include <time.h>
#include <string>
#include <stdlib.h>
//#include <pthread.h>
#include <limits>
#include <fstream>
#include <ostream>
#include <istream>

// AL Libraries
#include <alerror/alerror.h>
#include <alproxies/almotionproxy.h>
#include <alproxies/alrobotpostureproxy.h>
#include <alproxies/almemoryproxy.h>
#include <alproxies/alsensorsproxy.h>
#include <alproxies/dcmproxy.h>
#include <alproxies/albasicawarenessproxy.h>
#include <qi/os.hpp>
#include <alproxies/altexttospeechproxy.h>
#include <alproxies/alredballtrackerproxy.h>

// Custom Libraries

// Robotlib Libraries
#include <rlib_sensor.hpp>
#include <naopp.hpp>


///*** Main Function ***///
int main(int argc, char* argv[]) {
	if(argc != 2){
	    std::cerr << "Wrong number of arguments!" << std::endl;
	    std::cerr << "Usage: naocrawl NAO_IP" << std::endl;
	    exit(2);
  	}

	///*** Initialize Program Control Global Variables ***///
	bool robotHalted = false; // Controls if robot should run or stop.


	///*** Initialize rlib Objects ***///

	// Create Crawler.
	naopp::Crawler * crawler = new naopp::Crawler(argv[1], 9559);

	// Create thread for tuning planner gains.

	///*** Initialize NaoQi String Names ***///
	///*** Package this into the implementation object for the Robot Actuator Subclass ***///

	// String for Button Values
	const std::string frontButton 	= "Device/SubDeviceList/Head/Touch/Front/Sensor/Value";
	const std::string middleButton 	= "Device/SubDeviceList/Head/Touch/Middle/Sensor/Value";	
	const std::string rearButton 	= "Device/SubDeviceList/Head/Touch/Rear/Sensor/Value";	

  	///*** Initialize NaoQi Proxies ***///

	// Initialize Memory Proxy to grab memory values.
	AL::ALMemoryProxy memPrx(argv[1], 9559);
	// Initialize Motion Proxy for walking.
	AL::ALMotionProxy motionPrx(argv[1], 9559);
	// Initialize Posture Proxy for going to predefined poses.
	AL::ALRobotPostureProxy pstrPrx(argv[1], 9559);
	// Initialize Redball Tracking Proxy.
	AL::ALRedBallTrackerProxy redBallPrx(argv[1], 9559);
	// Initialize DCM Proxy, used to get time.
	AL::DCMProxy dcm_proxy(argv[1], 9559);
  
	///*** Configure NaoQi Proxies ***///
  
  
	// Disable Collision Protection

	// Head Pressed? Then crouch.
	while(memPrx.getData(frontButton) != AL::ALValue(1.0));

	// Crouch Nao.
	pstrPrx.goToPosture("LyingBelly", 1.0f);
	// Start tracking.

	// Initialize Crawl.
	if(crawler->initCrawl()){
		std::cout << "Crawl initialized." << std::endl;
	}

	try {

    	std::cout << "Loop started." << std::endl;

    	// Should the robot stop?
    	while(!robotHalted){

			// Head Pressed? Then stand.
			if(memPrx.getData(frontButton) == AL::ALValue(1.0)){
				// Stand Nao.
				pstrPrx.goToPosture("Stand", 1.0f);
			}
			if(memPrx.getData(rearButton) == AL::ALValue(1.0)){
				// Stand Nao.
				pstrPrx.goToPosture("Crouch", 1.0f);
			}


    		///*** Update System State ***///
			// Grab scan

			// Head Pressed?
			if(memPrx.getData(middleButton) == AL::ALValue(1.0)) robotHalted = true;

			crawler->crawl(1);
			// Update Motion Planner.
			// Update Ball Pose


    	}

    	///*** Shut things down ***///
		std::cout << "Robot Halted." << std::endl;
		
		// Stop robot.
    	// pstrPrx.goToPosture("LyingBelly", 1.0f);	
    	motionPrx.rest();
    	// Stop tracking.

    	// Terminate Lidar.

		// Wait for thread to terminate.

  	}
	catch (const AL::ALError& e) {
  		// Stop tracking.
    	// Stop Lidar.
    	exit(1);
  	}

  	exit(0);
}





