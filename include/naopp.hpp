// Nao Projected Profile Crawl Gait library v0.1
//
// Projected Profile is a gaiting scheme that uses the
// sagittal projection of the robot in order to produce
// a sequence of joint motions that make the robot crawl.
//
// Author: Griswald Brooks
// Email: griswald.brooks@gmail.com
//

#ifndef _PP_CRAWL_HPP
#define _PP_CRAWL_HPP

// Standard includes.
#include <vector>
#include <string>
// #include <ostream>

// AL Libraries
#include <alproxies/almotionproxy.h>
#include <alproxies/alrobotpostureproxy.h>

// Robotlib includes.

// Special includes.

// Thresholds for pose comparisons in radians and meters.
#define _NAOPP_POSE_ANGULAR_EPSILON 	0.02
#define _NAOPP_POSE_LINEAR_EPSILON		0.05

// Leg joint limits in radians.
#define _NAO_ANKLE_MAX 		 0.922747
#define _NAO_ANKLE_MIN		-1.189516
#define _NAO_KNEE_MAX		 2.112528
#define _NAO_KNEE_MIN		-0.092346
#define _NAO_HIP_MAX		 0.484090
#define _NAO_HIP_MIN		-1.535889

// Arm joint limits in radians.
#define _NAO_SHOULDER_PITCH_MAX		 2.0857 
#define _NAO_SHOULDER_PITCH_MIN		-2.0857
#define _NAO_SHOULDER_ROLL_MAX		 0.3142 
#define _NAO_SHOULDER_ROLL_MIN		-1.3265
#define _NAO_ELBOW_YAW_MAX			 2.0857 
#define _NAO_ELBOW_YAW_MIN			-2.0857
#define _NAO_ELBOW_ROLL_MAX			 1.5446 
#define _NAO_ELBOW_ROLL_MIN			 0.0349

// Link Lengths of the Nao in meters
// Body Length is calculated as 
// body_length = -Neck-to-shoulder + NeckOffsetZ + HipOffsetZ
// from the Nao documentation
#define BODY_LENGTH 	(-0.023 + 0.1265 + 0.085)
#define THIGH_LENGTH 	(0.1)
// humerus_length = 0.105;
// Length of humerus when projected onto the saggital plane
#define PROJECTED_HUMERUS_LENGTH 	(0.08975)
#define TIBIA_LENGTH 				(0.1029)

// Foot length in this context is calculated as the line from 
// the ankle to tip of foot, FootHeight in the Nao documentation
// and the measured sole length
#define FOOT_LENGTH		sqrt(pow(0.04519,2) + pow(0.0935,2))

// The distance in mm from where the toes touch the ground
// to where the elbows touch the ground. This distance is fixed.
#define XD	0.43

/*! \brief Nao Projected Profile namespace. */
namespace naopp{
	void testArmPose(int argc, char* argv[]);

	/*! \brief Class for storing crawl configurations. */
	class NaoCrawlPose{
	public:
		float anklePitch, kneePitch, hipPitch;
		float shoulderPitch, shoulderRoll; 
		float elbowRoll, elbowYaw;
		// R R R x
  		// R R R y
  		// R R R z
  		// 0 0 0 1
		std::vector<float> leftArmTf, rightArmTf;
	};

	std::ostream& operator<<(std::ostream& os, NaoCrawlPose& pose);

	/*! \brief Class for executing the Projected Profile Crawl Gait. */
	class Crawler{
	public:
		/*! \brief Constructor for Crawler object. 
		* \param[in] ip 	IP address of the Nao.
		* \param[in] port 	Port number used to connect to the Nao.	
		*/
		Crawler(const std::string &ip, int port=9559);

		/*! \brief Method to initialize crawling by going to ALRobotPosture LyingBelly. */
		bool initCrawl();

		/*! \brief Method to have the robot crawl.
		* \param[in] n 		The number of times the crawl should be performed.
		*/
		void crawl(int n);

	private:
		/*! \brief Method to set the robot to a certain crawl pose. Robot will try to go there directly
		*			without going to intermediate crawl poses.
		* \param[in] pose 	Crawl pose for the robot to go to.
		*/
		void _goto(NaoCrawlPose & pose);

		/*! \brief Method to compare the current pose to some desired pose. Compares joints angles
		*				of ankle, knee, hip, and position of hands.
		* \param[in] pose 	Pose to compare.
		* \return 			True if current pose is within _NAOPP_POSE_ANGULAR_EPSILON 
		*					and _NAOPP_POSE_LINEAR_EPSILON of pose.
		*/
		bool _comparePose(NaoCrawlPose & pose);

		/*! \brief Method to update current crawl pose by asking the robot.
		*/
		void _updatePose();

		/*! \brief Method to get the configuration angles as a function of time.
		* \param[in] t 		Time in seconds.
		* \param[out] cfg 	The configuration for that time.
		*					Format is [theta3, theta4, alpha].
		*/
		void _getCfg(float t, std::vector<float> & cfg);

		/*! \brief Method to convert configuration angles to full Projected Profile angles.
		* \param[in] cfg 		Set of configuration angles. 
		*						Format is [theta3, theta4, alpha].
		* \param[out] ppAngles 	Set of corresponding Projected Profile angles.
		*						Format is [theta1, theta2, theta3, theta4, theta5]
		*/
		void _cfg2PP(std::vector<float> & cfg, std::vector<float> & ppAngles);

		/*! \brief Method for converting Projected Profile angles to a Crawl Pose.
		* \param[in] ppAngles 	Set of Projected Profile angles to convert.
		*						Format is [theta1, theta2, theta3, theta4, theta5]
		* \param[out] pose 		Crawl pose for the Nao.
		*/
		void _pp2Nao(std::vector<float> & ppAngles, NaoCrawlPose & pose);

		/*! \brief Method to ensure the joints are commanded only within feasible joint limits.
		* \param[in] 			Pose to be checked. Method with clamp angles to joint limits
		*						if they exceed the range.
		*/
		void _enforceJointLimits(NaoCrawlPose & pose);

		/*! \brief Method to limit a number to a range, inclusive.
		* \param[in] n 		The number to be limited.
		* \param[in] min 	The minimum value for the range.
		* \param[in] max 	The maximum value for the range
		* \return 			The limited number.
		*/
		float _clamp(float n, float min, float max);

		AL::ALMotionProxy * _motionPrx; 		/**< Motion Proxy for commanding joints. */
		AL::ALRobotPostureProxy * _pstrPrx; 	/**< Motion Proxy for commanding joints. */

		int _armBaseFrame;		/**< Coordinate axis by which the arm will be positioned with respect to. */
		int _axisMask; 			/**< Controls the IK mode for the end effector (position only, orientation only,
																				or both). */
		bool _useSensorValues;	/**< Controls if sensors are used to return arm positions. */
		std::string _endLeft;	/**< String name for the left arm end effector (hand). */
		std::string _endRight;	/**< String name for the right arm end effector (hand). */

		float _jointSpeed; 	/**< How fast the joints should move. Range [0,1]. */
		float _armSpeed; 	/**< How fast the arms should move. Range [0,1]. */

		std::vector<float> _alphaCoeff; 	/**< Coefficients for spline interpolation of pp angle alpha. */
		std::vector<float> _theta3Coeff; 	/**< Coefficients for spline interpolation of pp angle theta3. */
		std::vector<float> _theta4Coeff; 	/**< Coefficients for spline interpolation of pp angle theta4. */

		float _ccTime;		/**< Normalized time parameter for the closed chain phase of the gait. */

		NaoCrawlPose _poseCurrent;						/**< Current pose of the robot. */
		NaoCrawlPose _poseExtension, _poseCompression; 	/**< Intermediate poses between open chain and closed
																chain gait configurations. */
		NaoCrawlPose _poseProne, _poseSuperman;	 		/**< Intermediate poses. */

	};
}

#endif
