#include "PID_Gimble_2M.h"

PID_Gimble_2M::PID_Gimble_2M(){
	topMotor = Motor(TOP_BIN2, TOP_BIN1, TOP_BPWM, offsetB, STBY);
	btmMotor = Motor(BTM_AIN1, BTM_AIN2, BTM_APWM, offsetA, STBY);
	myPID_btm = PID(&input_btm, &output_btm, &setpoint_btm, kp_btm, ki_btm, kd_btm, DIRECT);  
	myPID_top = PID(&input_top, &output_top, &setpoint_top, kp_top, ki_top, kd_top, DIRECT);  
	pinMode(TOP_B_ENCODER_A, INPUT_PULLUP);
	pinMode(TOP_B_ENCODER_B, INPUT_PULLUP);
	pinMode(BTM_A_ENCODER_A, INPUT_PULLUP);
	pinMode(BTM_A_ENCODER_B, INPUT_PULLUP);

	//on interrupt 0 (pin 2), or interrupt 1 (pin 3) 
	attachInterrupt(TOP_B_ENCODER_A_INT, &PID_Gimble_2M::updateTopEncoder, this, CHANGE); 
	attachInterrupt(TOP_B_ENCODER_B_INT, &PID_Gimble_2M::updateTopEncoder, this, CHANGE);
	attachInterrupt(BTM_A_ENCODER_A_INT, &PID_Gimble_2M::updateBtmEncoder, this, CHANGE); 
	attachInterrupt(BTM_A_ENCODER_B_INT, &PID_Gimble_2M::updateBtmEncoder, this, CHANGE);

	TCCR1B = TCCR1B & 0b11111000 | 1;  // set 31KHz PWM to prevent motor noise

	myPID_btm.SetMode(AUTOMATIC);   //set PID in Auto mode
	myPID_btm.SetSampleTime(1);  // refresh rate of PID controller
	myPID_btm.SetOutputLimits(-75, 75); // this is the MAX PWM value to move motor, here change in value reflect change in speed of motor.

	myPID_top.SetMode(AUTOMATIC);   //set PID in Auto mode
	myPID_top.SetSampleTime(1);  // refresh rate of PID controller
	myPID_top.SetOutputLimits(-90, 90); // this is the MAX PWM value to move motor, here change in value reflect change in speed of motor.

	delay(1000);

}

void PID_Gimble_2M::moveMotorTop(double degree){
	setpoint_top = map (User_Input_top, 0, 360, 0, _topMtrPPR); // mapping degree into pulse
	int error = INT_MAX;
	int errorCount = 0;
	while( errorCount < numMinError){
		input_top = _topEncoderCount ;  
		myPID_top.Compute();                 // calculate new output
		if (Math.abs(output_top - setpoint_top) < minError)
		{
			errorCount++;
		}
		topMotor.drive(output_top)
	}	
	topMotor.brake();
}

void PID_Gimble_2M::moveMotorBtm(double degree){
	setpoint_btm = map (User_Input_btm, 0, 360, 0, _btmMtrPPR); // mapping degree into pulse
	int errorCount = 0;
	while( errorCount < numMinError){
		input_btm = _btmEncoderCount ;  
		myPID_btm.Compute();                 // calculate new output
		if (Math.abs(output_btm - setpoint_btm) < minError)
		{
			errorCount++;
		}
		btmMotor.drive(output_btm);
	}	         
	btmMotor.brake();
}


void PID_Gimble_2M::updateTopEncoder(){
	int MSB = bitRead(PIND, TOP_B_ENCODER_A_PN); //MSB = most significant bit
	int LSB = bitRead(PIND, TOP_B_ENCODER_B_PN); //LSB = least significant bit
	int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
	int sum  = (lastTopEncoded << 2) | encoded; //adding it to the previous encoded value
	if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) _topEncoderCount --;
	if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) _topEncoderCount ++;
	//  _topEncoderCount ++
	lastTopEncoded = encoded; //store this value for next time
}

void PID_Gimble_2M::updateBtmEncoder(){
	int MSB = bitRead(PIND, BTM_A_ENCODER_A_PN); //MSB = most significant bit
	int LSB = bitRead(PIND, BTM_A_ENCODER_B_PN); //LSB = least significant bit
	int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
	int sum  = (lastBtmEncoded << 2) | encoded; //adding it to the previous encoded value
	if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) _btmEncoderCount ++;
	if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) _btmEncoderCount --;
	//  _topEncoderCount ++
	lastBtmEncoded = encoded; //store this value for next time
}
