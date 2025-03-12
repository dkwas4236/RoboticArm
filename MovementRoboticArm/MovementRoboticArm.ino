#include <AccelStepper.h>
#include <Servo.h>

// Motor connections to RAMPS 1.4
AccelStepper baseMotor(AccelStepper::DRIVER, 54, 55); // + right, - left
AccelStepper shoulderMotor(AccelStepper::DRIVER, 46, 48);  // - backward, + forward
AccelStepper elbowMotor(AccelStepper::DRIVER, 60, 61);  // + down, - up
 
int L2 = 140;
int L1 = 140;
double hthetaNot = 0;
double htheta1 = 20.92;
double htheta2 = 138.15;


struct Position {
    double x;
    double y;
    double z;
};

Position home {-100, 0, 0};
Position Dee5 {-15, 205, 20};

void setup() {

  // Setup enables for motors
  pinMode(38, OUTPUT);
  digitalWrite(38, LOW);
  pinMode(56, OUTPUT);
  digitalWrite(56, LOW);
  pinMode(62, OUTPUT);
  digitalWrite(62, LOW);
  
  Serial.begin(9600);
  while (!Serial) {
    ;
  }
  
  pinMode(9, OUTPUT);
  digitalWrite(9, HIGH);

  baseMotor.setMaxSpeed(500);
  baseMotor.setAcceleration(200);
  shoulderMotor.setMaxSpeed(500);
  shoulderMotor.setAcceleration(200);
  elbowMotor.setMaxSpeed(500);
  elbowMotor.setAcceleration(200);

  elbowMotor.setPinsInverted(true, false, false);
  baseMotor.setPinsInverted(true, false, false);
  delay(2000);
  moveF(0, 85.81, 20.92, 41.97, 138.15, 84.95);
}

// passed
double findThetaNot(double x, double y){
  double r = sqrt((sq(x)+sq(y)));
 
  double thetaNot = acos((-x/r)) * (180/PI);
  int steps = thetaNot * 40;
  //baseMotor.moveTo(steps);
  //while(baseMotor.distanceToGo() != 0){
  //  baseMotor.run();
  //}
  return thetaNot;
}

// passed
void move90degreesTest(){
  int steps = 3600;
  int stepcounter = 0;
  baseMotor.moveTo(steps);
  Serial.println(steps);
  while(baseMotor.distanceToGo() != 0){
    baseMotor.run();
    for (int i = 0; i >= steps; i++){
      stepcounter -= 1;
    }
  }
  Serial.println(stepcounter);
}

void moveF(double thetaNoti, double thetaNotf,double theta1i, double theta1f, double theta2i, double theta2f){
  double deltaThetaNot = thetaNotf - thetaNoti;
  double deltaTheta1 = theta1f - theta1i;
  double deltaTheta2 = theta2f - theta2i + deltaTheta1;
  
  int stepsThetaNot = deltaThetaNot * 40;
  int stepsTheta1 = deltaTheta1 * 40;
  int stepsTheta2 = deltaTheta2 * 40;

  baseMotor.move(stepsThetaNot);
  elbowMotor.move(stepsTheta2);
  shoulderMotor.move(stepsTheta1);

  while (baseMotor.distanceToGo() != 0) {
    baseMotor.run();
  }
  while (elbowMotor.distanceToGo() != 0){
    elbowMotor.run();
  }
  while (shoulderMotor.distanceToGo() != 0){
    shoulderMotor.run();
  }
  

  
}

void loop() {
  // no action needed
}
