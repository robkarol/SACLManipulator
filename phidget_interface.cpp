#include "phidget_interface.h"
#include "stdio.h"

int CCONV AttachHandler(CPhidgetHandle HANDLE, void *userptr) {
	int serialNo;
	const char *name;

	CPhidget_getDeviceName(HANDLE, &name);
	CPhidget_getSerialNumber(HANDLE, &serialNo);
	printf("\t%s %10d attached!\n", name, serialNo);
	return 0;
}

int CCONV DetachHandler(CPhidgetHandle HANDLE, void *userptr) {
	int serialNo;
	const char *name;

	CPhidget_getDeviceName(HANDLE, &name);
	CPhidget_getSerialNumber(HANDLE, &serialNo);
	printf("\t%s %10d detached!\n", name, serialNo);
	return 0;
}

int CCONV ErrorHandler(CPhidgetHandle HANDLE, void *userptr, int ErrorCode, const char *Description) {
	printf("\tError handled. %d - %s\n", ErrorCode, Description);
	return 0;
}

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


double Deg2Command(double q_deg) {
	double q_cmd = q_deg + 20.0;
	return q_cmd;
}

double Command2Deg(double q_cmd) {
	double q_deg = q_cmd - 20.0;
	return q_deg;
}