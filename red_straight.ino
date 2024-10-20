#include <Wire.h>
#include <Encoder.h>
#include <math.h>
#include <EasyTransfer.h>
#include "Adafruit_TCS34725.h"
#include <EasyTransfer.h>
#include <math.h>
#define DegreeToRadian(x) x * 0.0174532
float current_angle = 0;

EasyTransfer ET;
int bit = 0;
int set=0;
double u ;
double v ;
int num =0;


// int motor1d = 12;  // Motor pin 1
// int motor1p = 13;
// int motor2d = 11;  // Motor pin 2
// int motor2p = 10;
// int motor3d = 48;  // Motor pin 3
// int motor3p = 7;
// int motor4d = 6;  // Motor pin 4
// int motor4p = 5;
int armP = 9;
int armD = 52;
int PWM1[4] = { 13, 10, 7, 5 };
int DIR[4] = { 12, 11, 48, 6 };

int rollerP = 8;
int rollerD = 50;

int uln_1A = 1;
int uln_2A = 0;
int ir1 = 23;
int ir2 = 25;
int ir3 = 27;
// int num = 0;

long duration1, duration2;
long distance1, distance2;

int count = 0;
char command;

// Rotary encoder pins
const int encoderPinA = 20;
const int encoderPinB = 21;

const int encoderPinA1 = 18;
const int encoderPinB1 = 19;

// Create an Encoder object
Encoder myEncoder(encoderPinA, encoderPinB);
Encoder myEncoder1(encoderPinA1, encoderPinB1);
// Variables
volatile long prevPosition_x = 0;
volatile long prevPosition_y = 0;
unsigned long prevMicros = 0;
double prdistance_x = 0;
double prdistance_y = 0;

double target_distance_y;
double target_distance_x;
double targetAngle = 0 * 0.0174532;
double deldis = 0;
double delangle = 0;



#define DegreeToRadian(x) x * 0.0174532
// float current_angle = 0;
//create object

struct RECEIVE_DATA_STRUCTURE {
  //put your variable definitions here for the data you want to receive
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  float angle;
  int16_t pause;
};

//give a name to the group of data
RECEIVE_DATA_STRUCTURE mydata;
void setMotor(int dir, int pwmVal, int pwm, int in1) {
  analogWrite(pwm, pwmVal);  // Motor speed
  if (dir == 1) {
    // Turn one way
    digitalWrite(in1, HIGH);
  } else if (dir == -1) {
    // Turn the other way
    digitalWrite(in1, LOW);
  }}

void setup() {
  Serial.begin(115200);
  Serial3.begin(115200);
  ET.begin(details(mydata), &Serial3);

  pinMode(uln_1A, OUTPUT);
  pinMode(uln_2A, OUTPUT);
  pinMode(ir1, INPUT);
  pinMode(ir2, INPUT);
  pinMode(ir3, INPUT);


    for (int i = 0; i < 4; i++) {
    pinMode(PWM1[i], OUTPUT);
    pinMode(DIR[i], OUTPUT);
  }

  // pinMode(motor1p, OUTPUT);
  // pinMode(motor2p, OUTPUT);
  // pinMode(motor3p, OUTPUT);
  // pinMode(motor4p, OUTPUT);
  // pinMode(motor1d, OUTPUT);
  // pinMode(motor2d, OUTPUT);
  // pinMode(motor3d, OUTPUT);
  // pinMode(motor4d, OUTPUT);
  pinMode(armD, OUTPUT);
  pinMode(armP, OUTPUT);
  pinMode(rollerD, OUTPUT);
  pinMode(rollerP, OUTPUT);
  // Serial.println("O");
}


void loop() {
  // Serial.println("into loop");
  int ir1value = digitalRead(ir1);
  int ir2value = digitalRead(ir2);
  int ir3value = digitalRead(ir3);
  // Serial.println("into the void loop");
  if (bit == 0) {
    float current_angle;
    if (ET.receiveData()) {
    //this is how you access the variables. [name of the group].[variable name]
    //since we have data, we will blink it out.
    current_angle = mydata.angle;
    // Serial.println("angle is received");
    // Serial.println(current_angle);
  }
  
    // Read the current position from the encoder


    // Calculate RPM
    unsigned long currentMicros = millis();
    unsigned long elapsedMicros = currentMicros - prevMicros;

    if (elapsedMicros >= 100) {

      // float rpm_x = (deltaPosition_x / 10000.0) * 60.0;
    noInterrupts();
    long currentPosition_y = myEncoder1.read();
    long currentPosition_x = myEncoder.read();
    interrupts();


      long deltaPosition_x = currentPosition_x - prevPosition_x;
      double distance_x = (currentPosition_x / 10000.0) * (3.14159) * (0.06);

      double deltadistance_x = distance_x - prdistance_x;

      // Serial.println(distance_x);

      prevPosition_x = currentPosition_x;
      prdistance_x = distance_x;

      long deltaPosition_y = currentPosition_y - prevPosition_y;
      double distance_y = (currentPosition_y / 10000.0) * (3.14159) * (0.06);
      // float rpm_y = (deltaPosition_y / 10000.0) * 60.0;
      double deltadistance_y = distance_y - prdistance_y;


      // Serial.println(distance_y);

      prevPosition_y = currentPosition_y;
      prdistance_y = distance_y;

      prevMicros = currentMicros;

           // /------------------------------------LOCALIZATION-----------------------------------------------///
    delangle = atan2(deltadistance_x,deltadistance_y) - current_angle;

    deldis = sqrt((deltadistance_x * deltadistance_x) + (deltadistance_y * deltadistance_y));
    distance_x = distance_x + deldis * sin(delangle);
    distance_y = distance_y + deldis * cos(delangle);


      /*---------------------------------------targets---------------------------------
      ---------------------------------------------------------------------------------|
      ----------------------------------------------------------------------------------|*/
      if ((u > -0.05 && u < 0.05) && (v > -0.05 && v < 0.05)) {
        num = num + 1;
        // Serial.println("Change target");
      }


      // Switch case for different target distances
      switch (num) {
        case 1:
          // First target distance
          target_distance_x = 5.8;
          target_distance_y = 0;
          targetAngle = 20*0.0174532;
          break;
        case 2:
          // First target distance
          target_distance_x = 5.8;
          target_distance_y = 3.85;
          targetAngle = 20*0.0174532;
          break;
        case 3:
          // Second target distance
          // Serial.println("second_target");

          target_distance_x = 9.68;
          target_distance_y = 3.85;
          targetAngle = 0*0.0174532;
          // delay(500);
          break;
        // case 4:
        //   // Serial.println("third_target");

        //   target_distance_x = 5.5;
        //   target_distance_y = -2;
        //   targetAngle = -0*0.0174532;
        //   break;

        case 4:
          // Serial.println("fourth_target");
          target_distance_x = 9.68;
          target_distance_y = 3.85;
          targetAngle = 90*0.0174532;
          // bit=1;
          // set=1;
          // Serial.println("O");
          break;

        case 5:
          for (int i = 0; i < 4; i++) {
            analogWrite(PWM1[i],0);
          }
          bit = 1;
          set=1;
          Serial.println("O");
          break;
          // case 6:
          //   bit = 1;
          //   set=1;
          //   Serial.println("O");
          //   break;

      // Add more cases for additional target distances as needed

      // Serial.println(num);
      }


      double kp = 50.0;
      double kpi = 50;
      double kd = 0.9;
      double kdi = 0.9; 
      double theta = current_angle * 0.0174532;
      double e_x[4];
      double e_y[4];
      double e_prev_x[4];
      double e_prev_y[4];
      double velocity_x[4];
      double velocity_y[4];
      double derivativeError_x[4];
      double derivativeError_y[4];  // Declare derivativeError_y array
      double dedt[4];
      double velocity_drive[4];     // Declare velocity_drive array before usage
      // float x=0;
      // while( x <= 15*2000){
      // x = x + 0.01;
      // }
      double errorOmega[4];
      double Omega[4] = {0,0,0,0};
      double prevOmega[4];
      // float target_distance_x = 3.6;
      // float target_distance_y = 0;
      // float targetAngle = atan2(target_distance_y, target_distance_x);
      //  target_distance_x = -2.5;
      //  target_distance_y = 0;
      //  targetAngle = 0*0.0174532;
      //-------------------------------------------------------------------------------------------------------------/
      // // changing in the coordinates such it goes same coordinate every time ;
      // float target_in_x = dista5nce_x + (target_distance_x * cos(theta)) + (target_distance_y * sin(theta));
      // float target_in_y = distance_y - (target_distance_x * sin(theta)) + (target_distance_y * cos(theta));
      //-----------------------------------------------------------------------------------------------------------/
      // Calculate velocities and errors for each motor
      for (int i = 0; i < 4; i++) {
        e_x[i] = (target_distance_x - distance_x);
        derivativeError_x[i] = (e_x[i] - e_prev_x[i]);
        velocity_x[i] = (kp * e_x[i]) + kd * derivativeError_x[i];
        e_prev_x[i] = e_x[i] ;

        e_y[i] = (target_distance_y - distance_y);  // Fixed targetdistance_x to targetdistance_y
        derivativeError_y[i] = (e_y[i] - e_prev_y[i]); // Fixed e_x to e_y
        velocity_y[i] = (2*kp * e_y[i]) + kd * derivativeError_y[i]; // Fixed derivativeError_x to derivativeError_y
        e_prev_y[i] = e_y[i] ;
        
        // errorOmega[i] = (targetAngle - theta);
        // dedt[i] = (Omega[i] - prevOmega[i]);
        Omega[i] = kpi * (targetAngle - theta); //+ kdi * dedt[i];
        // prevOmega[i] = Omega[i];

        u = e_x[i];
        v = e_y[i];
        // Serial.print("x!!");
        // Serial.println(u);
        // Serial.print("y!!");
        // Serial.println(v);
      }

      velocity_drive[0] = (velocity_x[0] + velocity_y[0] - Omega[0]);
      velocity_drive[1] = (velocity_x[1] - velocity_y[1] + Omega[1]);
      velocity_drive[2] = (velocity_x[2] + velocity_y[2] + Omega[2]);
      velocity_drive[3] = (velocity_x[3] - velocity_y[3] - Omega[3]);

      int dir[4] = { 1, 1, 1, 1 };
      for (int i = 0; i < 4; i++) {
        if (velocity_drive[i] < 0) {
          dir[i] = -1;
        }
      }

      int maxpwr = 100;
      int minpwr = 25;
      // if ((u > -1 && u < 1) && (v > -1 && v < 1)) {
      //  maxpwr=50;
      // }
      // else{
      //   maxpwr=200;
      // }
      float pwr[4];
      for (int i = 0; i < 4; i++) {
        pwr[i] = (int)fabs(velocity_drive[i]);
        if (pwr[i] > maxpwr) {
          pwr[i] = maxpwr;
         }
           else if ((u > -0.5&& u < 0.5) && (v > -0.5 && v < 0.5)) {
          pwr[i] = minpwr;
          // Serial.print("|sexy| ");
          }

        //   else if ((u > -0.05 && u < 0.05) && (v > -0.05 && v < 0.05)) {
        //   pwr[i] = 0;
        //   Serial.print("|sexy| ");
        // } 
      }

      for (int i = 0; i < 4; i++) {
        setMotor(dir[i], pwr[i], PWM1[i], DIR[i]);
      }
      // for (int i = 0; i < 4; i++) {
      //   Serial.print("|x|--");
      //   Serial.print(i);
      //   Serial.print(" ");
      //   Serial.println(velocity_x[i]);
      //   Serial.print("|y|--");
      //   Serial.print(i);
      //   Serial.print(" ");
      //   Serial.println(velocity_y[i]);
      //   Serial.println(pwr[i]);
      //   Serial.print("|velocity_drive|");
      //   Serial.println(velocity_drive[i]);
      //   Serial.print("x");
      //   Serial.println(e_x[0]);
      //   Serial.print("y");
      //   Serial.println(e_y[0]);
      // }
      // Serial.println(current_angle);
      // Serial.println("current_angle:");
      // Serial.println(current_angle);
      // // Serial.println(target_in_x);
      // // Serial.println(target_in_y);
      // // for (int i = 0; i < 4; i++) {
      // //   Serial.println(Omega[i]);
      // // }
      // Serial.println(Omega[0]);
      // // delay(1000);
    }
  
}


// void rotate(double targetangle, double kpi, double current_angle){
//   for (int i = 0; i < 4; i++) {
//       double Omega[i] = kpi * (targetAngle - theta); 
//   }
//       double theta = current_angle * 0.0174532;

//       velocity_drive[0] = - Omega[0];
//       velocity_drive[1] = + Omega[1];
//       velocity_drive[2] = + Omega[2];
//       velocity_drive[3] = - Omega[3];
// }
    
  


  if(bit == 1){
  //ball ander hai 
  if (set == 2) {
    if(ir1value==HIGH){
    set=1;
    delay(300);
    Serial.println("O");
    
  }
    // decide=true;
    // Serial.println("P");
  
  }

  // ball nahi hai 
  if(set==1){
    if(ir1value==LOW){
      delay(200);
    Serial.println("P");
    set=2;

  }



    // Serial.println("O");

  }
   double velocity_x[4];
   double velocity_y[4];
   double velocity_drive[4];
      velocity_drive[0] = velocity_x[0] + velocity_y[0] ;
      velocity_drive[1] = velocity_x[1] - velocity_y[1] ;
      velocity_drive[2] = velocity_x[2] + velocity_y[2] ;
      velocity_drive[3] = velocity_x[3] - velocity_y[3] ;

  if (Serial.available() > 0) {
    command = Serial.read();
    Serial.println(command);
      if (command == 'f') {
        velocity_drive[0] = 60;
        velocity_drive[1] =60;
        velocity_drive[2] =60;
        velocity_drive[3] =60;
    } else if (command == 'b') {
        velocity_drive[0] =(-10);
        velocity_drive[1] =(-10);
        velocity_drive[2] =(-10);
        velocity_drive[3] =(-10);
      digitalWrite(armD, LOW);
      analogWrite(armP, 255);
      digitalWrite(rollerD, LOW);
      analogWrite(rollerP, 255);
      // delay(3000);
      // Serial.print("backward");
    }else if (command == 'r') {
        velocity_drive[0] =0;
        velocity_drive[1] =60;
        velocity_drive[2] =60;
        velocity_drive[3] =0;
      // Serial.println("P");
    } else if (command == 'l') {
        velocity_drive[0] =60;
        velocity_drive[1] =0;
        velocity_drive[2] =0;
        velocity_drive[3] =60;
    } else if (command == 's') {
        velocity_drive[0] =40;
        velocity_drive[1] =40;
        velocity_drive[2] =40;
        velocity_drive[3] =40;

      digitalWrite(armD, LOW);
      analogWrite(armP, 255);
      digitalWrite(rollerD, LOW);
      analogWrite(rollerP, 255);
      // x = true;
      // delay(5000);
      // rollersRunning = true;
    }
      else if (command == 'F') {  // Opposite of 'F'
        velocity_drive[0] =(-60);
        velocity_drive[1] =(-60);
        velocity_drive[2] =(-60);
        velocity_drive[3] =(-60);
    } else if (command == 'B') {  // Opposite of 'B'
        velocity_drive[0] =20;
        velocity_drive[1] =20;
        velocity_drive[2] =20;
        velocity_drive[3] =20;
    } else if (command == 'L') {  // Opposite of 'L'
        velocity_drive[0] = 0;
        velocity_drive[1] =(-60);
        velocity_drive[2] =(-60);
        velocity_drive[3] =0;
    } else if (command == 'R') {  // Opposite of 'R'

        velocity_drive[0] = (-60);
        velocity_drive[1] =0;
        velocity_drive[2] =0;
        velocity_drive[3] =(-60);
    } else if (command == 'S') {  // Opposite of 'S' and integrates ultrasonic logic
      // digitalWrite(motor1d, HIGH);
      // analogWrite(motor1p, 10);
      // digitalWrite(motor2d, HIGH);
      // analogWrite(motor2p, 10);
      // digitalWrite(motor3d, HIGH);
      // analogWrite(motor3p, 10);
      // digitalWrite(motor4d, HIGH);
      // analogWrite(motor4p, 10);
        velocity_drive[0] = 10;
        velocity_drive[1] =10;
        velocity_drive[2] =10;
        velocity_drive[3] =10;
    } else {
      // digitalWrite(motor1d, HIGH);
      // analogWrite(motor1p, 0);
      // digitalWrite(motor2d, HIGH);
      // analogWrite(motor2p, 0);
      // digitalWrite(motor3d, HIGH);
      // analogWrite(motor3p, 0);
      // digitalWrite(motor4d, HIGH);
      // analogWrite(motor4p, 0);
        velocity_drive[0] = 0;
        velocity_drive[1] =0;
        velocity_drive[2] =0;
        velocity_drive[3] =0;
    }

  }
  if(ir1value == LOW){
  if(ir2value == LOW && ir3value == LOW){
      // digitalWrite(motor1d, HIGH);
      // analogWrite(motor1p, 0);
      // digitalWrite(motor2d, HIGH);
      // analogWrite(motor2p, 0);
      // digitalWrite(motor3d, HIGH);
      // analogWrite(motor3p, 0);
      // digitalWrite(motor4d, HIGH);
      // analogWrite(motor4p, 0);
      digitalWrite(armD, LOW);
      analogWrite(armP, 0);
      digitalWrite(rollerD, LOW);
      analogWrite(rollerP, 0);
        velocity_drive[0] = 0;
        velocity_drive[1] =0;
        velocity_drive[2] =0;
        velocity_drive[3] =0;

    // Activate pneumatics
    analogWrite(uln_1A, 0);
    analogWrite(uln_1A, 255);
    analogWrite(uln_2A, 255);
    analogWrite(uln_2A, 0);
    analogWrite(uln_1A, 255);
    analogWrite(uln_1A, 0);
    analogWrite(uln_2A, 0);
    analogWrite(uln_2A, 255);
    delay(500);
  }
  }
  }
}