

int BIN_1 = 9;
int BIN_2 = 7;
int AIN_1 = 17;
int AIN_2 = 4;
int pwma = 13;
int pwmb = 5;
int standby = 15;
int MAX_PWM_VOLTAGE = 20;

void setup() {Serial.begin(9600);
    pinMode(BIN_1, OUTPUT);
    pinMode(BIN_2, OUTPUT);
    pinMode(AIN_1, OUTPUT);
    pinMode(AIN_2, OUTPUT);
     pinMode(pwma, OUTPUT);
      pinMode(pwmb, OUTPUT);
       pinMode(standby, OUTPUT);
           digitalWrite(standby, HIGH);

}

void loop() {
//  if (Serial.available() > 0) {
    // get incoming byte:
//   MAX_PWM_VOLTAGE = Serial.parseInt();

    digitalWrite(BIN_2, LOW);
    digitalWrite(AIN_2, LOW);
    analogWrite(BIN_1, MAX_PWM_VOLTAGE);
    analogWrite(AIN_1, MAX_PWM_VOLTAGE);
    Serial.println(MAX_PWM_VOLTAGE);
//  }
    delay(2000);
    
//    digitalWrite(BIN_1, LOW);
//    digitalWrite(AIN_2, LOW);
//    analogWrite(BIN_2, MAX_PWM_VOLTAGE);
//    analogWrite(AIN_1, MAX_PWM_VOLTAGE);
//    delay(2000);
//    
//    digitalWrite(BIN_2, LOW);
//    digitalWrite(AIN_1, LOW);
//    analogWrite(BIN_1, MAX_PWM_VOLTAGE);
//    analogWrite(AIN_2, MAX_PWM_VOLTAGE);
//    delay(2000);
//    
//    digitalWrite(BIN_1, LOW);
//    digitalWrite(AIN_1, LOW);
//    analogWrite(BIN_2, MAX_PWM_VOLTAGE);
//    analogWrite(AIN_2, MAX_PWM_VOLTAGE);
//    delay(2000);
}
