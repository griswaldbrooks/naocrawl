// Nao Projected Profile Crawl Gait library v0.1
//
// Projected Profile is a gaiting scheme that uses the
// sagittal projection of the robot in order to produce
// a sequence of joint motions that make the robot crawl.
//
// Author: Griswald Brooks
// Email: griswald.brooks@gmail.com
//

// Include header.
#include <naopp.hpp>

// Standard includes.
#include <vector>
#include <string>
#include <ostream>
#include <math.h>

// AL Libraries
#include <alproxies/almotionproxy.h>
#include <alproxies/alrobotpostureproxy.h>
#include <alvalue/alvalue.h>
#include <almath/types/altransform.h>

// Robotlib includes.

// Special includes.

// Projected Profile
// File format: [time, theta1, theta2, theta3, theta4, theta5]
// On Nao: 		[time, foot-ground angle, ankle, knee, hip, shoulder]
// The Projected Profile angles need to be converted to Nao angles.

#define DEG2RAD(x) 		(M_PI/180.0)*x

/*! \brief Nao Projected Profile namespace. */
namespace naopp{

	void testArmPose(int argc, char* argv[]){
		// Initialize Motion Proxy for walking.
		AL::ALMotionProxy _motionPrx(argv[1], 9559);
		// Initialize Posture Proxy for going to predefined poses.
		AL::ALRobotPostureProxy pstrPrx(argv[1], 9559);

	    // Stand Nao.
		pstrPrx.goToPosture("StandInit", 1.0f);

	    std::string effector   = "LArm";
	    int frame      = 0; // FRAME_TORSO
	    // int axisMask   = 7; // just control position
	    int axisMask   = 63; // just control position
	    bool useSensorValues = false;

	    // std::vector<std::vector<float> > path;
	    // std::vector<float> currentTf = _motionPrx.getTransform(effector, frame, useSensorValues);
	    // std::vector<float> targetTf  = currentTf;
	    std::vector<float> targetTf(12,0.0f);
	    targetTf.at(3) = 0.1; // x
	    targetTf.at(7) = 0.2; // y
	    targetTf.at(11) = 0.3; // z

	    targetTf.at(8) = 1.0; // wx
	    targetTf.at(5) = 1.0; // wy
	    targetTf.at(2) = -1.0; // wz

	    // path.push_back(targetTf);
	    // path.push_back(currentTf);

	    // Go to the target and back again
	    // float times[]      = {2.0, 4.0}; // seconds

	    // _motionPrx.transformInterpolations(effector, frame, path, axisMask, times);
	    _motionPrx.setTransforms(effector, frame, targetTf, 0.2f, axisMask);
	    sleep(15.0);

	    targetTf.at(11) = 0.2; // z
	    _motionPrx.setTransforms(effector, frame, targetTf, 0.2f, axisMask);
	    sleep(15.0);
	    // Go to rest position
	    _motionPrx.rest();
	}

	// // Convert to method.
	// void ppToNao(std::vector<float> ppAngles){
	// 	///** Map the projected profile angles to Nao ***///
		
	// 	// Both ankles get theta2
	// 	// Ankles are offset by 52 degrees (some tuning required/done)
	// 	naoAnklePitch = ppAngles(2) + deg2rad(60);

	// 	// Both knees get theta3
	// 	naoKneePitch = ppAngles(3);

	// 	// Both hips get theta4
	// 	naoHipPitch = ppAngles(4);

	// 	///*** Compute the arm angles ***///
	// 	// Both shoulders get theta5
	// 	// Shoulders are offset by 90 degrees
	// 	// naoShoulderPitch = -pp_Angles(5) - pi/2;

	// 	///*** Compute arm end effector target ***///


	// }

	std::ostream& operator<<(std::ostream& os, NaoCrawlPose& pose){
		os << "[";
		os << pose.leftArmTf[3] << ", ";
		os << pose.leftArmTf[7] << ", ";
		os << pose.leftArmTf[11] << ", ";
		os << pose.rightArmTf[3] << ", ";
		os << pose.rightArmTf[7] << ", ";
		os << pose.rightArmTf[11] << ", ";
		os << pose.anklePitch << ", ";
		os << pose.kneePitch << ", ";
		os << pose.shoulderPitch << ", ";
		os << pose.shoulderRoll << ", ";
		os << pose.elbowYaw << ", ";
		os << pose.elbowRoll << "]";
	}

	Crawler::Crawler(const std::string &ip, int port){
		// Initialize NaoQi Proxies
		_motionPrx 	= new AL::ALMotionProxy(ip, port);
		_pstrPrx 	= new AL::ALRobotPostureProxy(ip, port);

		// Initial configurations [\alpha, \theta3, \theta4]
		// x_init = [-30*pi/180 ; 0.28798  ;   0.47997];

		// Best x value found after a few runs of the genetic algorithm
		// x_best = [ -1.9362   -0.8890    1.7781   -0.0872   -0.0099    0.0972    0.3388   -0.6390    0.3002];
		// x_best = [-0.2134    1.1570   -1.9898    0.2365    0.0893   -0.3267    1.8796   -0.1365   -1.7434];
		// optimized trajectory
		// x1_ = x_best(1)*t.^3 + x_best(2)*t.^2 + x_best(3)*t + x_init(1);
		// x2_ = x_best(4)*t.^3 + x_best(5)*t.^2 + x_best(6)*t + x_init(2);
		// x3_ = x_best(7)*t.^3 + x_best(8)*t.^2 + x_best(9)*t + x_init(3);
		_alphaCoeff.push_back(-0.2134); _alphaCoeff.push_back( 1.1570); _alphaCoeff.push_back(-1.9898); _alphaCoeff.push_back(DEG2RAD(-30.0));
		// _theta3Coeff.push_back(0.2365); _theta3Coeff.push_back( 0.0893); _theta3Coeff.push_back(-0.3267); _theta3Coeff.push_back(0.28798);
		// _theta4Coeff.push_back(1.8796); _theta4Coeff.push_back(-0.1365); _theta4Coeff.push_back(-1.7434); _theta4Coeff.push_back(0.47997);
		_theta3Coeff.push_back(0); _theta3Coeff.push_back(0); _theta3Coeff.push_back(0); _theta3Coeff.push_back(0.28798);
		_theta4Coeff.push_back(0); _theta4Coeff.push_back(0); _theta4Coeff.push_back(0); _theta4Coeff.push_back(0.47997);

		// Set arm base from to torso frame.
		// FRAME_TORSO = 0
		// FRAME_WORLD = 1
		// FRAME_ROBOT = 2
		_armBaseFrame = 0;

		// Set control mode to position and orientation of end effector.
		// AXIS_MASK_X 1
		// AXIS_MASK_Y 2
		// AXIS_MASK_Z 4
		// AXIS_MASK_WX 8
		// AXIS_MASK_WY 16
		// AXIS_MASK_WZ 32
		// Position Only = 7
		// Orientation Only = 56
		// Both = 63
	    // _axisMask   = 63; 
	    _axisMask   = 1 + 2 + 4 + 8 + 16 + 32; 
	    // Use the sensors to check arm positions.
	    _useSensorValues = true;

		// Set end effect string names.
		_endLeft   = "LArm";
		_endRight  = "RArm";

		// Set the default speed, which is a percentage of max speed from [0,1].
		_armSpeed = 0.2f;
		_jointSpeed = 0.2f;

		///*** Set Prone Pose. ***///
		_poseProne.anklePitch 	= DEG2RAD(-65.0);
		_poseProne.kneePitch	= DEG2RAD(120.0);
		_poseProne.hipPitch		= DEG2RAD(0.0);

		// Temp vector.
		std::vector<float> targetTf(12,0.0f);

		// Points arm forwards.
	    targetTf.at(8) = 1.0; // wx
	    targetTf.at(5) = 1.0; // wy
	    targetTf.at(2) = -1.0; // wz

	    // Puts hand near head.
	    targetTf.at(3) = 0.04; // x
	    targetTf.at(7) = 0.2; // y
	    targetTf.at(11) = 0.2; // z

	    // Set hands.
	    _poseProne.leftArmTf = targetTf;
	    // Switch to right side.
	    targetTf.at(7) = -targetTf.at(7); // y
	    _poseProne.rightArmTf = targetTf;

	    ///*** Set Superman Pose. ***///
	    _poseSuperman.anklePitch 	= DEG2RAD(-65.0);
		_poseSuperman.kneePitch		= DEG2RAD(120.0);
		_poseSuperman.hipPitch		= DEG2RAD(0.0);
		
		// Puts hands behind head.
		//-0.0167871, 0.155986, 0.282275
	    targetTf.at(3) 	= -0.015; // x
	    targetTf.at(7) 	=  0.15; // y
	    targetTf.at(11) =  0.3; // z

		_poseSuperman.leftArmTf = targetTf;
		targetTf.at(7) = -targetTf.at(7); // y
	    _poseSuperman.rightArmTf = targetTf;

	    ///*** Set Extension Pose. ***///
		_poseExtension.anklePitch 	= -2.223 + DEG2RAD(60.0);
		_poseExtension.kneePitch 	= 0.28798;
		_poseExtension.hipPitch		= 0.47997;

		// Puts hand in front of head.
		// 0.0332247, 0.119067, 0.298142
	    targetTf.at(3) = 0.03; // x
	    targetTf.at(7) = 0.1; // y
	    targetTf.at(11) = 0.3; // z

		_poseExtension.leftArmTf = targetTf;
		targetTf.at(7) = -targetTf.at(7); // y
	    _poseExtension.rightArmTf = targetTf;


		///*** Set Compression Pose. ***///
		// -1.3912, 0.28798, 0.47997
		_poseCompression.anklePitch	= -1.3912 + DEG2RAD(60.0);
		_poseCompression.kneePitch 	= 0.28798;
		_poseCompression.hipPitch	= 0.47997;

		// Puts hand in front of head.
		// 0.111837, 0.160225, 0.221327
	    targetTf.at(3) = 0.1; // x
	    targetTf.at(7) = 0.15; // y
	    targetTf.at(11) = 0.2; // z

		_poseCompression.leftArmTf = targetTf;
		targetTf.at(7) = -targetTf.at(7); // y
	    _poseCompression.rightArmTf = targetTf;

		// Get current pose.
		_updatePose();
	}

	/*! \brief Method to initialize crawling by going to ALRobotPosture LyingBelly. */
	bool Crawler::initCrawl(){
		// Init configuration parameters.

		// Set head and hips.
		AL::ALValue headAndHipNames 	= AL::ALValue::array("HeadYaw","LHipYawPitch", "RHipYawPitch");
		AL::ALValue headAndHipAngles 	= AL::ALValue::array(0.0f,0.0f,0.0f);
		_motionPrx->setAngles(headAndHipNames, headAndHipAngles, _jointSpeed);

		// Set closed chain time to zero to indicate the beginning of the phase.
		_ccTime = 0;

		// Set Robot to its belly.
		if(_pstrPrx->goToPosture("LyingBelly", 1.0f)){
			// Goto Extension Pose.
			// _goto(_poseSuperman);
			// _goto(_poseProne);
			// _goto(_poseExtension);
		}
		else{
			return false;
		}
	}

	/*! \brief Method to have the robot crawl.
	* \param[in] n 		The number of times the crawl should be performed.
	*/
	void Crawler::crawl(int n){
		// Closed Chain Variables.
		std::vector<float> cfg(3,0.0f), ppAngles(5,0.0f);
		NaoCrawlPose ccPose;		
		char x;
		// Start the loop.
		for(int i = 0; i < n; i++){

			// Extension Pose.
			std::cout << "Crawl: Going to Extension." << std::endl;
			std::cin >> x;
			// _goto(_poseExtension);

			// Pull forward/Closed Chain.
			for(float t = 0.0f; t < 1.0f; t += 0.05){
				// Get the current configuration.
				_getCfg(t, cfg);
				// Get the Projected Profile angles.
				_cfg2PP(cfg, ppAngles);
				// Get the Nao Pose.
				_pp2Nao(ppAngles, ccPose);
				// Set the pose.
				std::cout << "Pose: ";
				std::cout << _poseCurrent;
				std::cout << std::endl;
				std::cout << "ccPose: ";
				std::cout << ccPose;
				std::cout << std::endl;
				std::cout << "Crawl: Going to Pose." << std::endl;
				std::cin >> x;
				_goto(ccPose);
				
			}

			// Compression Pose.
			std::cout << "Crawl: Going to Compression." << std::endl;
			std::cin >> x;
			_goto(_poseCompression);

			// Prone Pose.
			std::cout << "Crawl: Going to Prone." << std::endl;
			std::cin >> x;
			_goto(_poseProne);

		}
		
	}

	/*! \brief Method to get the configuration angles as a function of time.
	* \param[in] t 		Time in seconds.
	* \param[out] cfg 	The configuration for that time.
	*					Format is [theta3, theta4, alpha].
	*/
	void Crawler::_getCfg(float t, std::vector<float> & cfg){
		// Configuration parameters.
		float alpha, theta3, theta4;
		// Optimized trajectory from genetic splines.
		alpha 	= _alphaCoeff[0]*pow(t,3)  + _alphaCoeff[1]*pow(t,2)  + _alphaCoeff[2]*t  + _alphaCoeff[3];
		theta3 	= _theta3Coeff[0]*pow(t,3) + _theta3Coeff[1]*pow(t,2) + _theta3Coeff[2]*t + _theta3Coeff[3];
		theta4 	= _theta4Coeff[0]*pow(t,3) + _theta4Coeff[1]*pow(t,2) + _theta4Coeff[2]*t + _theta4Coeff[3];

		// Add them to the vector.
		cfg.clear();
		cfg.push_back(theta3);cfg.push_back(theta4);cfg.push_back(alpha);
	}

	/*! \brief Method to convert configuration angles to full Projected Profile angles.
	* \param[in] cfg 		Set of configuration angles. 
	*						Format is [theta3, theta4, alpha].
	* \param[out] ppAngles 	Set of corresponding Projected Profile angles.
	*						Format is [theta1, theta2, theta3, theta4, theta5]
	*/
	void Crawler::_cfg2PP(std::vector<float> & cfg, std::vector<float> & ppAngles){
		// Closed chain lengths.
		float l1, l2, l3, l4, l5;
		// Closed chain angles.
		float theta1, theta2, theta3, theta4, theta5, alpha;	
		// IK variables.
		float x_hat, y_hat;
		float k1, k2, k3, k4, k5;
		float A, B, C;
		float tan_gamma, costheta1, sintheta1;

		// Set the configurable joint variables from cfg
		theta3 = cfg[0];
		theta4 = cfg[1];
		alpha  = cfg[2];	

		// Assign the link lengths.
		l1 = FOOT_LENGTH;
		l2 = TIBIA_LENGTH;
		l3 = THIGH_LENGTH;
		l4 = BODY_LENGTH;
		l5 = PROJECTED_HUMERUS_LENGTH;

		// Use the Inverse Kinematics of the constrained profile to 
		// compute the other joint angles
		x_hat = XD - l5*cos(alpha); 
		y_hat = -l5*sin(alpha);
		k1 = l2 + l3*cos(theta3) + l4*cos(theta3 + theta4);
		k2 = -l3*sin(theta3) - l4*sin(theta3 + theta4);
		k5 = (pow(x_hat,2) + pow(y_hat,2) - pow(l1,2) - pow(k1,2) - pow(k2,2))/(2*l1);
		// Parameters for solving quadratic 
		A = -k1 - k5;
		B = 2*k2;
		C = k1 - k5;

		// tan_gamma = roots([A,B,C]);
		tan_gamma = (-B + sqrt(pow(B,2) - 4*A*C))/(2*A);
		// theta2 = 2*atan(tan_gamma(1));
		theta2 = 2*atan(tan_gamma);

		k3 = l1 + k1*cos(theta2) + k2*sin(theta2);
		k4 = k2*cos(theta2) - k1*sin(theta2);
		costheta1 = (k3*x_hat - k4*y_hat)/(pow(k3,2) + pow(k4,2));
		sintheta1 = (k4*x_hat + k3*y_hat)/(pow(k3,2) + pow(k4,2));
		theta1 = atan2(sintheta1,costheta1);
		theta5 = alpha - theta1 - theta2 - theta3 - theta4;

		// Set the output variables.
		ppAngles.clear();
		ppAngles.push_back(theta1); 
		ppAngles.push_back(theta2);
		ppAngles.push_back(theta3); 
		ppAngles.push_back(theta4); 
		ppAngles.push_back(theta5);

		std::cout << "Angles: [";
		for(int i = 0; i < ppAngles.size(); i++)
			std::cout << ppAngles[i] << " ";
		 std::cout << std::endl;
	}

	/*! \brief Method for converting Projected Profile angles to a Crawl Pose.
	* \param[in] ppAngles 	Set of Projected Profile angles to convert.
	*						Format is [theta1, theta2, theta3, theta4, theta5]
	* \param[out] pose 		Crawl pose for the Nao.
	*/
	void Crawler::_pp2Nao(std::vector<float> & ppAngles, NaoCrawlPose & pose){
		// Ankles are offset by 52 degrees (some tuning required/done).
		pose.anklePitch = ppAngles[1] + DEG2RAD(60);

		// Both knees get theta3.
		pose.kneePitch = ppAngles[2];

		// Both hips get theta4.
		pose.hipPitch = ppAngles[3];

		pose.shoulderPitch = DEG2RAD(-90) + ppAngles[4];

		pose.shoulderRoll = DEG2RAD(0.0);

		// pose.elbowRoll = DEG2RAD(180.0) - (DEG2RAD(180.0) + ppAngles[0] + ppAngles[1] + ppAngles[2] + ppAngles[3] + ppAngles[4]);
		pose.elbowRoll = -(ppAngles[0] + ppAngles[1] + ppAngles[2] + ppAngles[3] + ppAngles[4]);

		pose.elbowYaw = DEG2RAD(-90.0); // Left is -90.0, Right is 90.0

		// Ensure the joint angles are feasible.
		_enforceJointLimits(pose);

 // [0.0326261, 0.1, 0.299726, 0.0326261, -0.1, 0.299726, -1.25659, 0.28798, -0.689374, 0, -1.5708, 0.523599]

		// Arm end effector poses are a function of the forward kinematics.
		// Closed chain lengths.
		float l1, l2, l3, l4, l5;
		// Assign the link lengths.
		l1 = FOOT_LENGTH;
		l2 = TIBIA_LENGTH;
		l3 = THIGH_LENGTH;
		l4 = BODY_LENGTH;
		l5 = PROJECTED_HUMERUS_LENGTH;

		///*** Find initial shoulder position. ***///
		std::vector<float>  initCfg; 
		std::vector<float>  ppInitAngles;
		float dx, xc, x0;
		// Load initial configuration.
		initCfg.push_back(_theta3Coeff[3]); initCfg.push_back(_theta4Coeff[3]); initCfg.push_back(_alphaCoeff[3]);
		// Get initial angles.
		_cfg2PP(initCfg, ppInitAngles);

		// Find initial shoulder position.
		x0 = 	l1*cos(ppInitAngles[0]) + 
				l2*cos(ppInitAngles[0]  + ppInitAngles[1]) + 
				l3*cos(ppInitAngles[0]  + ppInitAngles[1]  + ppInitAngles[2]) + 
				l4*cos(ppInitAngles[0]  + ppInitAngles[1]  + ppInitAngles[2]  + ppInitAngles[3]);

		// Find current shoulder position.
		xc = 	l1*cos(ppAngles[0]) + 
				l2*cos(ppAngles[0]  + ppAngles[1]) + 
				l3*cos(ppAngles[0]  + ppAngles[1]  + ppAngles[2]) + 
				l4*cos(ppAngles[0]  + ppAngles[1]  + ppAngles[2]  + ppAngles[3]);

		// Change in x position.
		dx = xc - x0;

		// Compute new pose.
		pose.leftArmTf  = _poseExtension.leftArmTf;
		pose.rightArmTf = _poseExtension.rightArmTf;
		// Subtract the change in shoulder position.
		pose.leftArmTf[11]  -= dx;
		pose.rightArmTf[11] -= dx;

		// New arm angle wrt the torso.
		float armAngle = -ppAngles[0] - ppAngles[1] - ppAngles[2] - ppAngles[3] + DEG2RAD(10.0);
		pose.leftArmTf  = (AL::Math::Transform::fromRotY(armAngle)*AL::Math::Transform(pose.leftArmTf)).toVector();
		pose.rightArmTf = (AL::Math::Transform::fromRotY(armAngle)*AL::Math::Transform(pose.rightArmTf)).toVector();

	}

	/*! \brief Method to set the robot to a certain crawl pose. Robot will try to go there directly
	*			without going to intermediate crawl poses.
	* \param[in] pose 	Crawl pose for the robot to go to.
	*/
	void Crawler::_goto(NaoCrawlPose & pose){
	    // Set arms.
	    // _motionPrx->setTransforms(_endLeft, _armBaseFrame, pose.leftArmTf, _armSpeed, _axisMask);
	    // _motionPrx->setTransforms(_endRight, _armBaseFrame, pose.rightArmTf, _armSpeed, _axisMask);

		// Arm names.
		AL::ALValue leftArmNames  = AL::ALValue::array("LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll");
		AL::ALValue rightArmNames = AL::ALValue::array("RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll");
		// Arm angles.
		AL::ALValue leftArmAngles  = AL::ALValue::array(pose.shoulderPitch, pose.shoulderRoll, pose.elbowYaw, -pose.elbowRoll);
		AL::ALValue rightArmAngles = AL::ALValue::array(pose.shoulderPitch, -pose.shoulderRoll, -pose.elbowYaw, pose.elbowRoll);

	    // Set arms.
	    _motionPrx->setAngles( leftArmNames,  leftArmAngles, _jointSpeed);
	    _motionPrx->setAngles(rightArmNames, rightArmAngles, _jointSpeed);

	    // Leg names.
		AL::ALValue leftLegNames = AL::ALValue::array("LAnklePitch","LKneePitch", "LHipPitch");
		AL::ALValue rightLegNames = AL::ALValue::array("RAnklePitch","RKneePitch", "RHipPitch");
		// Leg angles.
		AL::ALValue legAngles = AL::ALValue::array(pose.anklePitch, pose.kneePitch, pose.hipPitch);

	    // Set legs.
	    _motionPrx->setAngles(leftLegNames, legAngles, _jointSpeed);
	    _motionPrx->setAngles(rightLegNames, legAngles, _jointSpeed);
	    
	    // Check to see if we made it.
	    // while(!_comparePose(pose)) _updatePose();
	    sleep(1);
	}

	/*! \brief Method to compare the current pose to some desired pose. Compares joints angles
	*				of ankle, knee, hip, and position of hands.
	* \param[in] pose 	Pose to compare.
	* \return 			True if current pose is within _NAOPP_POSE_ANGULAR_EPSILON 
	*					and _NAOPP_POSE_LINEAR_EPSILON of pose.
	*/
	bool Crawler::_comparePose(NaoCrawlPose & pose){
		float errRHand, errLHand;
		float errShoulder, errElbow;
		float errAnkle, errKnee, errHip;

		// Compute hand errors.
		// errRHand = sqrt(pow(_poseCurrent.rightArmTf[3] -  pose.rightArmTf[3],2) + 
		// 				pow(_poseCurrent.rightArmTf[7] -  pose.rightArmTf[7],2) + 
		// 				pow(_poseCurrent.rightArmTf[11] - pose.rightArmTf[11],2)
		// 				);
		// errLHand = sqrt(pow(_poseCurrent.leftArmTf[3] -  pose.leftArmTf[3],2) + 
		// 				pow(_poseCurrent.leftArmTf[7] -  pose.leftArmTf[7],2) + 
		// 				pow(_poseCurrent.leftArmTf[11] - pose.leftArmTf[11],2)
		// 				);
		
		// Compute arm errors.
		// errShoulder = fabs(_poseCurrent.shoulderPitch 	- pose.shoulderPitch);
		// errElbow 	= fabs(_poseCurrent.elbowRoll 		- pose.elbowRoll);

		// Compute leg errors.
		errAnkle 	= fabs(_poseCurrent.anklePitch 	- pose.anklePitch);
		errKnee 	= fabs(_poseCurrent.kneePitch 	- pose.kneePitch);
		errHip 		= fabs(_poseCurrent.hipPitch 	- pose.hipPitch);

		// Are we close enough?
		if(	
			// errRHand 	> _NAOPP_POSE_LINEAR_EPSILON 	||
			// errLHand 	> _NAOPP_POSE_LINEAR_EPSILON 	||
			// errShoulder	> _NAOPP_POSE_ANGULAR_EPSILON 	||
			// errElbow 	> _NAOPP_POSE_ANGULAR_EPSILON 	||
			errAnkle 	> _NAOPP_POSE_ANGULAR_EPSILON 	||
			errKnee 	> _NAOPP_POSE_ANGULAR_EPSILON 	||
			errHip 		> _NAOPP_POSE_ANGULAR_EPSILON)
		{
			// std::cout << "Err: [";
			// std::cout << errLHand << ", ";
			// std::cout << errRHand << ", ";
			// std::cout << errAnkle << ", ";
			// std::cout << errKnee << ", ";
			// std::cout << errHip << "]";
			// std::cout << std::endl;
			return false;
		}

		// Did we make it?
		return true;

	}

	/*! \brief Method to update current crawl pose by asking the robot.
	*/
	void Crawler::_updatePose(){

		// Update the arms.
		_poseCurrent.leftArmTf = _motionPrx->getTransform(_endLeft, _armBaseFrame, _useSensorValues);
		_poseCurrent.rightArmTf = _motionPrx->getTransform(_endRight, _armBaseFrame, _useSensorValues);



		// Joint Names.
		AL::ALValue leftLegNames  = AL::ALValue::array("LAnklePitch","LKneePitch", "LHipPitch");
		AL::ALValue rightLegNames = AL::ALValue::array("RAnklePitch","RKneePitch", "RHipPitch");
		AL::ALValue leftArmNames  = AL::ALValue::array("LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll");
		AL::ALValue rightArmNames = AL::ALValue::array("RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll");
		
		std::vector<float> leftLegAngles, rightLegAngles, leftArmAngles, rightArmAngles;
		leftLegAngles  = _motionPrx->getAngles( leftLegNames, _useSensorValues);
		rightLegAngles = _motionPrx->getAngles(rightLegNames, _useSensorValues);
		leftArmAngles  = _motionPrx->getAngles( leftArmNames, _useSensorValues);
		rightArmAngles = _motionPrx->getAngles(rightArmNames, _useSensorValues);

		// Update the legs.
		_poseCurrent.anklePitch = 0.5*(leftLegAngles[0] + rightLegAngles[0]);
		_poseCurrent.kneePitch 	= 0.5*(leftLegAngles[1] + rightLegAngles[1]);
		_poseCurrent.hipPitch 	= 0.5*(leftLegAngles[2] + rightLegAngles[2]);

		// Update the arms.

		_poseCurrent.shoulderPitch = 0.5*(leftArmAngles[0] + rightArmAngles[0]);
		_poseCurrent.shoulderRoll  = 0.5*(leftArmAngles[1] - rightArmAngles[1]);
		_poseCurrent.elbowYaw 	   = 0.5*(leftArmAngles[2] - rightArmAngles[2]);
		_poseCurrent.elbowRoll 	   = 0.5*(-leftArmAngles[3] + rightArmAngles[3]);

		std::cout << "Pose: ";
		std::cout << _poseCurrent;
		std::cout << std::endl;
	}

	/*! \brief Method to ensure the legs are commanded only within feasible joint limits.
	* \param[in] 			Pose to be checked. Method with clamp angles to joint limits
	*						if they exceed the range.
	*/
	void Crawler::_enforceJointLimits(NaoCrawlPose & pose){
		// Check ankle.
		_clamp(pose.anklePitch, _NAO_ANKLE_MIN, _NAO_ANKLE_MAX);

		// Check knee.
		_clamp(pose.kneePitch, _NAO_KNEE_MIN, _NAO_KNEE_MAX);

		// Check hip.
		_clamp(pose.hipPitch, _NAO_HIP_MIN, _NAO_HIP_MAX);
		
		// Check arms.
		_clamp(pose.shoulderPitch,_NAO_SHOULDER_PITCH_MIN, _NAO_SHOULDER_PITCH_MAX);		
		_clamp(pose.shoulderRoll,_NAO_SHOULDER_ROLL_MIN,	_NAO_SHOULDER_ROLL_MAX);	
		_clamp(pose.elbowYaw,_NAO_ELBOW_YAW_MIN,	_NAO_ELBOW_YAW_MAX);		
		_clamp(pose.elbowRoll,_NAO_ELBOW_ROLL_MIN, _NAO_ELBOW_ROLL_MAX);		
	}

	/*! \brief Method to limit a number to a range, inclusive.
	* \param[in] n 		The number to be limited.
	* \param[in] min 	The minimum value for the range.
	* \param[in] max 	The maximum value for the range
	* \return 			The limited number.
	*/
	float Crawler::_clamp(float n, float min, float max){
		if(n > max) n = max;
		else if(n < min) n = min;

		return n;
	}

}

// Plus
// ccPose: [0.03, 		0.1, 	  0.3, 		0.03,  	   -0.1, 	  0.3, 		1.52717,  -0.523599,  0.408009]
// Pose: 	[0.0478084, 0.107876, 0.314602, 0.0456519, -0.105553, 0.315152, 0.922296, -0.0923279, 0.405743]


// Minus
// ccPose: [	0.03, 0.1, 		0.3, 	  0.03, 	 -0.1, 		0.3, 	  1.52717,  -0.523599,  -0.121614]
// Pose: [0.0189798, 0.113551, 0.318167, 0.0290675, -0.106377, 0.317871, 0.922296, -0.0912731, -0.120419]