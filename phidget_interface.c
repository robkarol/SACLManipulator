#include "phidget_interface.h"

/*+============================+*/
/*| Phidget control board code |*/
/*+============================+*/

/*! Declares a (global) Phidgets AdvancedServo handle */
CPhidgetAdvancedServoHandle servo	= 0;
/*! Declares a (global) Phidgets InterfaceKit handle */
CPhidgetInterfaceKitHandle ifKit	= 0;

/*! Code to execute when a Phidget device has been successfully attached. */
int CCONV AttachHandler(CPhidgetHandle HANDLE, void *userptr) {
	int serialNo;
	const char *name;

	CPhidget_getDeviceName(HANDLE, &name);
	CPhidget_getSerialNumber(HANDLE, &serialNo);
	printf("\t%s %10d attached!\n", name, serialNo);
	return 0;
}

/*! Code to execute when a Phidget device has been successfully attached. */
int CCONV DetachHandler(CPhidgetHandle HANDLE, void *userptr) {
	int serialNo;
	const char *name;

	CPhidget_getDeviceName(HANDLE, &name);
	CPhidget_getSerialNumber(HANDLE, &serialNo);
	printf("\t%s %10d detached!\n", name, serialNo);
	return 0;
}

/*! Code to execute when an error has occurred while interacting with a Phidget device. */
int CCONV ErrorHandler(CPhidgetHandle HANDLE, void *userptr, int ErrorCode, const char *Description) {
	printf("\tError handled. %d - %s\n", ErrorCode, Description);
	return 0;
}

/*! Attempts to connect to robotic arm servomotors and initializes the Phidget advanced servo global variable with user-defined values. \see ::servo
	\param[in] n				Number of channels (dimension of C-space)
	\param[in] channels			Channel numbers for joint servos \f$i = 1,\ldots,n\f$ with angles \f$q_i\f$
	\param[in] grip_channel		Channel corresponding to the end effector
	\param[in] AccelThrottle		Acceleration settings \f$\in [0,1]\f$ for each joint servo as a fraction from AccelMin to AccelMax
	\param[in] VelLimThrottle		Velocity limit settings \f$\in [0,1]\f$ for each joint servo as a fraction from VelMin to VelMax
	\param[in] grip_AccelThrottle	Acceleration setting \f$\in [0,1]\f$ for the end effector as a fraction from AccelMin to AccelMax
	\param[in] grip_VelLimThrottle	Velocity limit setting \f$\in [0,1]\f$ for the end effector as a fraction from VelMin to VelMax */
CPhidgetAdvancedServoHandle InitializeServos(int n, int* channels, int grip_channel, double* AccelThrottle, double* VelLimThrottle, double grip_AccelThrottle, double grip_VelLimThrottle) {
	int result;
	const char *err;
	double minAccel, maxAccel, minVel, maxVel;

	/* If servo is already defined, return. */
	if (servo != NULL) {
		return servo;
	}

	/* Create the advanced servo object */
	CPhidgetAdvancedServo_create(&servo);

	/* Set the handlers to be run when the device is plugged in (opened) from software, unplugged (closed) from software, or generates an error. */
	CPhidget_set_OnAttach_Handler((CPhidgetHandle)servo, AttachHandler, NULL);
	CPhidget_set_OnDetach_Handler((CPhidgetHandle)servo, DetachHandler, NULL);
	CPhidget_set_OnError_Handler((CPhidgetHandle)servo, ErrorHandler, NULL);

	/* Open the device for connections */
	CPhidget_open((CPhidgetHandle)servo, -1);

	printf("Waiting for servos to be attached...\n");
	if ((result = CPhidget_waitForAttachment((CPhidgetHandle)servo, 5000))) {
		CPhidget_getErrorDescription(result, &err);
		printf("\tProblem waiting for attachment: %s\n", err);
		return 0;
	}

	/* Set the acceleration and velocity limits of each joint angle actuator */
	for (int i = 0; i < n; i++) {
		CPhidgetAdvancedServo_getAccelerationMin(servo, channels[i], &minAccel);
		CPhidgetAdvancedServo_getAccelerationMax(servo, channels[i], &maxAccel);
		CPhidgetAdvancedServo_setAcceleration(servo, channels[i], AccelThrottle[i]*(maxAccel - minAccel) + minAccel );
		CPhidgetAdvancedServo_getVelocityMin(servo, channels[i], &minVel);
		CPhidgetAdvancedServo_getVelocityMax(servo, channels[i], &maxVel);
		CPhidgetAdvancedServo_setVelocityLimit(servo, channels[i], VelLimThrottle[i]*(maxVel - minVel) + minVel );
	}

	/* Set the acceleration and velocity limits of the end effector */
	CPhidgetAdvancedServo_getAccelerationMin(servo, grip_channel, &minAccel);
	CPhidgetAdvancedServo_getAccelerationMax(servo, grip_channel, &maxAccel);
	CPhidgetAdvancedServo_setAcceleration(servo, grip_channel, grip_AccelThrottle*(maxAccel - minAccel) + minAccel );
	CPhidgetAdvancedServo_getVelocityMin(servo, grip_channel, &minVel);
	CPhidgetAdvancedServo_getVelocityMax(servo, grip_channel, &maxVel);
	CPhidgetAdvancedServo_setVelocityLimit(servo, grip_channel, grip_VelLimThrottle*(maxVel - minVel) + minVel );

	return (CPhidgetAdvancedServoHandle)servo;
}

/*! Attempts to connect to the temperature sensor potentiometer board and initializes the Phidget interface kit global variable with user-defined values. \see ::ifKit
	\param[in]	n_tempsensors		Number of simulated temperature sensors (potentiometer channels)
	\param[in]	sensor_channels		Channel numbers for simulated sensors
	\param[in]	rate_tempsensors	Data rate (period in milliseconds) to use for reading temperatures */
CPhidgetInterfaceKitHandle InitializeTempSensors(int n_tempsensors, int* sensor_channels, int rate_tempsensors) {
	int result;
	const char *err;

	/* If ifKit is already defined, return. */
	if (ifKit != NULL) {
		return ifKit;
	}

	/* Create the InterfaceKit object */
	CPhidgetInterfaceKit_create(&ifKit);

	/* Set the handlers to be run when the device is plugged in or opened from software, unplugged or closed from software, or generates an error. */
	CPhidget_set_OnAttach_Handler((CPhidgetHandle)ifKit, AttachHandler, NULL);
	CPhidget_set_OnDetach_Handler((CPhidgetHandle)ifKit, DetachHandler, NULL);
	CPhidget_set_OnError_Handler((CPhidgetHandle)ifKit, ErrorHandler, NULL);

	/* Open the interfacekit for device connections */
	CPhidget_open((CPhidgetHandle)ifKit, -1);

	printf("Waiting for interface kit to be attached...\n");
	if((result = CPhidget_waitForAttachment((CPhidgetHandle)ifKit, 5000))) {
		CPhidget_getErrorDescription(result, &err);
		printf("\tProblem waiting for attachment: %s\n", err);
		return 0;
	}

	/* Set the data input rates of the interface kit potentiometers */
	for (int i = 0; i < n_tempsensors; i++) {
		CPhidgetInterfaceKit_setDataRate(ifKit, sensor_channels[i], rate_tempsensors);
	}

	return ifKit;
}

