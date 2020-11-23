#pragma once

#include "Arduino.h"
#include <SparkFun_TB6612.h>
#include "PID_v1.h"
#include "360GimbleConfig.h"
#include "limits.h"
#include "Math.h"

class PID_Gimble_2M
{
	public:
	PID_Gimble_2M();
	void moveMotorTop(double degree);
	void moveMotorBtm(double degree);

	private:
	// Motor objects
	Motor topMotor;
	Motor btmMotor;
	// PID objects
	PID myPID_btm;
	PID myPID_top;

	volatile int lastTopEncoded = 0; // Here updated value of encoder store.
	volatile int lastBtmEncoded = 0; // Here updated value of encoder store.
	volatile int _btmEncoderCount;
	volatile int _topEncoderCount;
	int REV_btm = 0;          // Set point REQUIRED ENCODER VALUE
	int REV_top = 0;          // Set point REQUIRED ENCODER VALUE
	int lastMSB = 0;
	int lastLSB = 0;
	// _btm motor Encoder Pulse per revolution. 
	const int _btmMtrPPR = 6330;  
	// _top motor Encoder Pulse per revolution.
	const int _topMtrPPR = 4540;  
	// pid values
	const double kp_btm =  8, ki_btm = 0.01 , kd_btm = 0.01;             // modify for optimal performance
	const double kp_top =  11.5, ki_top = 0.0125 , kd_top = 0.05;             // modify for optimal performance
	const minError = 10;
	const numMinError = 50;
	double input_btm = 0, output_btm = 0, setpoint_btm = 0;
	double input_top = 0, output_top = 0, setpoint_top = 0;
	void updateTopEncoder();
	void updateBtmEncoder()
};