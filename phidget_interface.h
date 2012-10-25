#ifndef __PHIDGET_INTERFACE_H__
#define __PHIDGET_INTERFACE_H__

#include <phidget21.h>	/* Library containing the commands used to control the Phidget servo-actuators */

/*! Declares a (global) Phidgets AdvancedServo handle */
static CPhidgetAdvancedServoHandle servo	= 0;
/*! Declares a (global) Phidgets InterfaceKit handle */
static CPhidgetInterfaceKitHandle ifKit	= 0;

/*! Code to execute when a Phidget device has been successfully attached. */
int CCONV AttachHandler(CPhidgetHandle HANDLE, void *userptr);

/*! Code to execute when a Phidget device has been successfully attached. */
int CCONV DetachHandler(CPhidgetHandle HANDLE, void *userptr);

/*! Code to execute when an error has occurred while interacting with a Phidget device. */
int CCONV ErrorHandler(CPhidgetHandle HANDLE, void *userptr, int ErrorCode, const char *Description);


/*! Attempts to connect to robotic arm servomotors and initializes the Phidget advanced servo global variable with user-defined values. \see ::servo
	\param[in] n				Number of channels (dimension of C-space)
	\param[in] channels			Channel numbers for joint servos \f$i = 1,\ldots,n\f$ with angles \f$q_i\f$
	\param[in] grip_channel		Channel corresponding to the end effector
	\param[in] AccelThrottle		Acceleration settings \f$\in [0,1]\f$ for each joint servo as a fraction from AccelMin to AccelMax 
	\param[in] VelLimThrottle		Velocity limit settings \f$\in [0,1]\f$ for each joint servo as a fraction from VelMin to VelMax 
	\param[in] grip_AccelThrottle	Acceleration setting \f$\in [0,1]\f$ for the end effector as a fraction from AccelMin to AccelMax
	\param[in] grip_VelLimThrottle	Velocity limit setting \f$\in [0,1]\f$ for the end effector as a fraction from VelMin to VelMax */
CPhidgetAdvancedServoHandle InitializeServos(int n, int* channels, int grip_channel, double* AccelThrottle, double* VelLimThrottle, double grip_AccelThrottle, double grip_VelLimThrottle);

/*! Attempts to connect to the temperature sensor potentiometer board and initializes the Phidget interface kit global variable with user-defined values. \see ::ifKit
	\param[in]	n_tempsensors		Number of simulated temperature sensors (potentiometer channels)
	\param[in]	sensor_channels		Channel numbers for simulated sensors
	\param[in]	rate_tempsensors	Data rate (period in milliseconds) to use for reading temperatures */
CPhidgetInterfaceKitHandle InitializeTempSensors(int n_tempsensors, int* sensor_channels, int rate_tempsensors);


/*! Convert a joint angle in degrees to its appropriate command value */
double Deg2Command(double q_deg);

/*! Convert a joint angle command back to degrees */
double Command2Deg(double q_cmd);

#endif