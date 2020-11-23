#pramga once

// Pins for all inputs, keep in mind the PWM defines must be on PWM pins
// the default pins listed are the ones used on the Redbot (ROB-12097) with
// the exception of STBY which the Redbot controls with a physical switch
const int PWMA 10  // Motor A
const int AIN1 14 //A0
const int AIN2 4  //
const int STBY 12   // Standby pin of TB6612. Shared by both channels
const int PWMB 5   // Motor B
const int BIN1 6
const int BIN2 7

// pins for the encoder inputs
const int BTM_A_ENCODER_A 0 
//use pin 7 
const int BTM_A_ENCODER_B 1 
const int BTM_A_ENCODER_A_PN 2 
const int BTM_A_ENCODER_B_PN 3 
const int BTM_A_ENCODER_A_INT 2 
const int BTM_A_ENCODER_B_INT 3 
const int BTM_AIN1 20
const int BTM_AIN2 21
const int BTM_APWM 10



const int TOP_B_ENCODER_A 2
const int TOP_B_ENCODER_B 3
const int TOP_B_ENCODER_A_PN 1
const int TOP_B_ENCODER_B_PN 0
const int TOP_B_ENCODER_A_INT 1
const int TOP_B_ENCODER_B_INT 0
const int TOP_BIN1 18
const int TOP_BIN2 19
const int TOP_BPWM 9

const int STBY 15

const int MotEnable 6 //Motor Enamble pin Runs on PWM signal
const int MotFwd  4  // Motorrr Forward pin
const int MotRev  7 // Motor Reverse pin

// these constants are used to allow you to make your motor configuration 
// line up with function names like forward.  Value can be 1 or -1
const int offsetA = 1;
const int offsetB = 1;
