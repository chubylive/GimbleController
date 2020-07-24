#include <PID_v1.h>
#include <SparkFun_TB6612.h>

// Pins for all inputs, keep in mind the PWM defines must be on PWM pins
// the default pins listed are the ones used on the Redbot (ROB-12097) with
// the exception of STBY which the Redbot controls with a physical switch
#define PWMA 10  // Motor A
#define AIN1 14 //A0
#define AIN2 4  //
#define STBY 12   // Standby pin of TB6612. Shared by both channels
#define PWMB 5   // Motor B
#define BIN1 6
#define BIN2 7

// pins for the encoder inputs
#define A_ENCODER_A 3 
#define A_ENCODER_B A1
#define B_ENCODER_A 2
#define B_ENCODER_B A2

#define MotEnable 6 //Motor Enamble pin Runs on PWM signal
#define MotFwd  4  // Motor Forward pin
#define MotRev  7 // Motor Reverse pin

// these constants are used to allow you to make your motor configuration 
// line up with function names like forward.  Value can be 1 or -1
const int offsetA = 1;
const int offsetB = 1;

// Initializing motors.  The library will allow you to initialize as many
// motors as you have memory for.  If you are using functions like forward
// that take 2 motors as arguements you can either write new functions or
// call the function more than once.
Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);


String readString; //This while store the user input data
int User_Input_btm = 0; // This while convert input string into integer
int User_Input_top = 0; // This while convert input string into integer
int encoderPin1 = 2; //Encoder Output 'A' must connected with intreput pin of arduino.
int encoderPin2 = 3; //Encoder Otput 'B' must connected with intreput pin of arduino.
volatile int lastEncoded = 0; // Here updated value of encoder store.
volatile long encoderValue = 0; // Raw encoder value
volatile long _btmEncoderCount;
volatile long _topEncoderCount;

const int _btmMtrPPR = 4040;  // _btm motor Encoder Pulse per revolution. TODO: update with not book numbers
const int _topMtrPPR = 2272;  // _btm motor Encoder Pulse per revolution.
int angle = 360; // Maximum degree of motion.
int REV_btm = 0;          // Set point REQUIRED ENCODER VALUE
int REV_top = 0;          // Set point REQUIRED ENCODER VALUE
int lastMSB = 0;
int lastLSB = 0;
double kp = 1.51, ki = 0.0009 , kd = 0.05;             // modify for optimal performance
double input_btm = 0, output_btm = 0, setpoint_btm = 0;
double input_top = 0, output_top = 0, setpoint_top = 0;

PID myPID_btm(&input_btm, &output_btm, &setpoint_btm, kp, ki, kd, DIRECT);  
PID myPID_top(&input_top, &output_top, &setpoint_top, kp, ki, kd, DIRECT);  

void setup() { 
  Serial.begin(9600); //initialize serial comunication
  pinMode(B_ENCODER_A, INPUT);
  pinMode(B_ENCODER_B, INPUT);
  pinMode(A_ENCODER_A, INPUT);
  pinMode(A_ENCODER_B, INPUT);
  //call updateEncoder() when any high/low changed seen
  //on interrupt 0 (pin 2), or interrupt 1 (pin 3) 
  attachInterrupt(0, _topEncoderEvent, RISING); 
  attachInterrupt(1, _btmEncoderEvent, RISING);
    
//  TCCR1B = TCCR1B & 0b11111000 | 1;  // set 31KHz PWM to prevent motor noise
  myPID_btm.SetMode(AUTOMATIC);   //set PID in Auto mode
  myPID_btm.SetSampleTime(1);  // refresh rate of PID controller
  myPID_btm.SetOutputLimits(-75, 75); // this is the MAX PWM value to move motor, here change in value reflect change in speed of motor.

  myPID_top.SetMode(AUTOMATIC);   //set PID in Auto mode
  myPID_top.SetSampleTime(1);  // refresh rate of PID controller
  myPID_top.SetOutputLimits(-50, 50); // this is the MAX PWM value to move motor, here change in value reflect change in speed of motor.
}


void loop() {
  
  while (Serial.available()) { //Check if the serial data is available.
    delay(3);                  // a small delay
    char c = Serial.read();  // storing input data
    readString += c;         // accumulate each of the characters in readString
  }
 
  if (readString.length() >0) { //Verify that the variable contains information
    String _btmVal = getValue(readString,',',0);
    String _topVal = getValue(readString,',',1);
//    Serial.println("_btm value: " + _btmVal.toInt());  //printing the input data in integer form
//    Serial.println("_top value: " + _topVal.toInt());  //printing the input data in integer form
    User_Input_btm = _btmVal.toInt();   // here input data is store in integer form
    User_Input_top = _topVal.toInt();   // here input data is store in integer form
  } 
  User_Input_top = 100;
REV_btm = map (User_Input_btm, 0, 360, 0, _btmMtrPPR); // mapping degree into pulse
REV_top = map (User_Input_top, 0, 360, 0, _topMtrPPR); // mapping degree into pulse
//Serial.print("this is REV _btm - "); 
//Serial.print(REV_btm);               // printing REV value 
//Serial.print("  this is REV _top - "); 
//Serial.println(REV_top);               // printing REV value  
setpoint_btm = REV_btm;                    //PID while work to achive this value consider as SET value
setpoint_top = REV_top;  
input_btm = _btmEncoderCount ;           
input_top = _topEncoderCount ;           
//Serial.print("_btm encoder Count - ");
//Serial.print(_btmEncoderCount);
//Serial.print("  _top encoder Count - ");
//Serial.println(_topEncoderCount);
//myPID_btm.Compute();                 // calculate new output TODO
myPID_top.Compute();                 // calculate new output
Serial.print(setpoint_top);
Serial.print("," );
Serial.print(input_top);
Serial.print(",");
Serial.print(output_top);
Serial.println("");
//write out to the motor
//motor1.drive(output_btm); //TODO
motor1.drive(output_top);
//Serial.print("_btm motor out - ");
//Serial.print(output_btm);
//Serial.print("  _top motor out - ");
//Serial.println(output_top);
}

String getValue(String data, char separator, int index)
{
    int found = 0;
    int strIndex[] = { 0, -1 };
    int maxIndex = data.length() - 1;

    for (int i = 0; i <= maxIndex && found <= index; i++) {
        if (data.charAt(i) == separator || i == maxIndex) {
            found++;
            strIndex[0] = strIndex[1] + 1;
            strIndex[1] = (i == maxIndex) ? i+1 : i;
        }
    }
    return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}
//void pwmOut(int out) {                               
//  if (out > 0) {                         // if REV > encoderValue motor move in forward direction.    
//    analogWrite(MotEnable, out);         // Enabling motor enable pin to reach the desire angle
//    forward();                           // calling motor to move forward
//  }
//  else {
//    analogWrite(MotEnable, abs(out));          // if REV < encoderValue motor move in forward direction.                      
//    reverse();                            // calling motor to move reverse
//  }
// readString=""; // Cleaning User input, ready for new Input
//}
//void updateEncoder(){
//  int MSB = digitalRead(encoderPin1); //MSB = most significant bit
//  int LSB = digitalRead(encoderPin2); //LSB = least significant bit
//  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
//  int sum  = (lastEncoded << 2) | encoded; //adding it to the previous encoded value
//  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue ++;
//  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue --;
//  lastEncoded = encoded; //store this value for next time
//}

// encoder event for the interrupt call
void _topEncoderEvent() {
  if (bitRead(PIND, 2) == HIGH) {
    if (bitRead(PINC, 2) == LOW) {
      _topEncoderCount++;
    } else {
      _topEncoderCount--;
    }
  } else {
    if (bitRead(PINC, 2) == LOW) {
      _topEncoderCount--;
    } else {
      _topEncoderCount++;
    }
  }
 }

// encoder event for the interrupt call
void _btmEncoderEvent() {

  if (bitRead(PIND, 3) == HIGH) {
    if (bitRead(PINC, 1) == LOW) {
      _btmEncoderCount++;
    } else {
      _btmEncoderCount--;
    }
  } else {
    if (bitRead(PINC, 1) == LOW) {
      _btmEncoderCount--;
    } else {
      _btmEncoderCount++;
    }
  }
}
