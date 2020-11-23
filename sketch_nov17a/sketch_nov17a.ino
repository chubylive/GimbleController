#include "PID_Gimble_2M.h"
PID_Gimble_2M gimble;
void updateTopEncoderEx();
void updateBtmEncoderEx();
void setup() {
  // put your setup code here, to run once:
  gimble.attachInterrupts(updateTopEncoderEx, updateBtmEncoderEx);
}

void loop() {
  // put your main code here, to run repeatedly:

}

void updateTopEncoderEx(){
  gimble.updateTopEncoder();
}

void updateBtmEncoderEx(){
  gimble.updateBtmEncoder();
}
