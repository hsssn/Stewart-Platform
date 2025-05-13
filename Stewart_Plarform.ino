#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <BasicLinearAlgebra.h>
#define SERVOMIN  125 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  640 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates]


using namespace BLA;

BLA::Matrix<3,6, double> base_points = {54.108,89.337, 35.228, -35.228, -89.337, -54.108,
                        -71.918, -10.9, 82.817, 82.817, -10.9, -71.918,
                        0,0,0,0,0,0}; // Coordinates of the Base Points based on the design.
BLA::Matrix<3,6, double> platform_points = {61.535, 72.769, 11.228, -11.228, -72.764, -61.535,
                                -48.493, -29.044, 77.537, 77.537, -29.654, -48.493,
                                89,89,89,89,89,89};  // Coordinates of the Platform Points based on the design.

const BLA::Matrix<1,6, double> beta = {PI/3, 7*PI/6, PI, 2*PI, 5*PI/3, 2*PI/3};
BLA::Matrix<1,6, double> z = {0, 0, 0, 0, 0, 0}; // Variable used in calculating angles
const double ldl = 75.0; // Servo Rod length.
const double lhl =  24.0; // Servo arm length                               
const BLA::Matrix<1,3, double> r_angles = {0,0, 0}; // Roll, Pitch, and Yaw angles, specified by the user.
const BLA::Matrix<3,1, double> T = {0,0,0}; // Transformation of the platform, specified by the user.
BLA::Matrix<3,3, double> rotx(double theta); // X-Rotation Matrix.
BLA::Matrix<3,3, double> roty(double psi); // Y-Rotation Matrix.
BLA::Matrix<3,3, double> rotz(double phi); // Z-Rotation Matrix.
BLA::Matrix<1,6, double> calculate_e (BLA::Matrix<3,6, double> l);
BLA::Matrix<3,6, double> calculate_l(BLA::Matrix<3,3, double>r_matrix, BLA::Matrix<3,1,double> T); // Calculate leg lengths.
BLA::Matrix<1,6, double> l_norm(BLA::Matrix<3,6, double> l); // Normalize l.
BLA::Matrix<1,6, double> get_angles (BLA::Matrix<3,6,double> leg_leng, BLA::Matrix<1,6,double> e, BLA::Matrix<1,6, double> gg); // calculate servo angles.
int angleToPulse(int ang); // PWM of Servo angles.

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); // Starting the servo driver.

void setup() {

  Serial.begin(9600);
  Serial.println("8 channel Servo test!");

  pwm.begin();
  pwm.setPWMFreq(60);
  
  double ldl_sq = ldl*ldl; // Rod length squared.
  double lhl_sq = lhl*lhl; // Servo arm length squared.
  
  for (int i=0; i < 6; i++){
    
    double aux_0 = platform_points(0,i) - base_points(0,i);
    double aux_0sq = aux_0 * aux_0;
    double aux_1 = platform_points(1,i) - base_points(1,i);
    double aux_1sq = aux_1 * aux_1;
    z(0,i)= sqrt(ldl_sq + lhl_sq - (aux_0sq) - (aux_1sq));
    
    // This block is to calculate z values.
  }
}

void loop() {

  double theta = r_angles(0,0);
  double psi = r_angles(0,1);
  double phi = r_angles(0,2);

  BLA::Matrix<3,3, double> rott = rotx(theta)*roty(psi);
  BLA::Matrix<3,3, double> rotation = rott*rotz(phi);
  BLA::Matrix<3,6, double> leg_leng = calculate_l(rotation, T);
  BLA::Matrix<1,6, double> lll = l_norm(leg_leng);
  
  BLA::Matrix<1,6, double> gg = {0,0,0,0,0,0};
  for (int i =0; i < 6; i++){
    gg(0,i) = lll(0,i)*lll(0,i) - ((ldl*ldl) - (lhl*lhl));
  }
  
  BLA::Matrix<1,6, double> e = {0,0,0,0,0,0};
  for (int i =0; i < 6; i++){
    e(0,i) = 2*lhl*leg_leng(2,i);
  }
  
  BLA::Matrix<1,6, double> angles = get_angles(leg_leng, e, gg);
  Serial.print(angles);
  Serial.println();
  
  for (int j =0; j <5; j++){
    if (j % 2 == 0){
      float ang = angles(0,j) * 180/PI;
      int p = angleToPulse(ang);
      pwm.setPWM(j, 0, p); 
    }
    else {
      float ang = 180 - (angles(0,j)*180/PI);
      int p = angleToPulse(ang);
      pwm.setPWM(j,0,p);     
    }
  }
  
  

}

BLA::Matrix<3,3, double> rotx(double theta){
  BLA::Matrix<3,3, double>rotx = {1,0,0,
                                  0, cos(theta*PI/180), sin(theta*PI/180),
                                  0, -sin(theta*PI/180), cos(theta*PI/180)};
return rotx;
}

BLA::Matrix<3,3, double> roty(double psi){
  BLA::Matrix<3,3, double>roty = {cos(psi*PI/180),0,sin(psi*PI/180),
                                  0, 1, 0,
                                  -sin(psi*PI/180), 0, cos(psi*PI/180)};
return roty;
}

BLA::Matrix<3,3, double> rotz(double phi){
  BLA::Matrix<3,3, double>rotz = {cos(phi*PI/180),-sin(phi*PI/180),0,
                                  sin(phi*PI/180), cos(phi*PI/180), 0,
                                  0, 0, 1};
return rotz;
}

BLA::Matrix<3,6, double> calculate_l(BLA::Matrix<3,3, double>r_matrix, BLA::Matrix<3,1,double> T){

  // Calculates required leg lengths to get to the required configuration, uses basic matrix transformations.
  
  BLA::Matrix<3,6, double> l;
  l.Fill(0);
  BLA::Matrix<3,6, double> aux = r_matrix*platform_points;
  for (int i=0; i < 3; i++){
    for (int j=0; j <6; j++){
      l(i,j) = T(i,0) + aux(i,j) - base_points(i,j);
    }
  }
  return l;
}

BLA::Matrix<1,6, double> l_norm(BLA::Matrix<3,6, double> l){

  // normalizes the leg lengths (using x, y, and z components of each leg).

  BLA::Matrix<1,6, double> norm_l = {0,0,0,0,0,0};
  
  for (int j=0; j <6; j++){
    double sum = 0.0;
    double sum_rt = 0.0;
    for (int i =0; i<3; i++){
      sum = sum + l(i,j)*l(i,j);
    }  
    sum_rt = sqrt(sum);
    norm_l(0,j) = sum_rt;
  }

  return norm_l;
}



BLA::Matrix<1,6, double> calculate_e (BLA::Matrix<3,6, double> l){
  BLA::Matrix<1,6, double> e = {0,0,0,0,0,0};
  for (int i =0; i < 6; i++){
    e(0,i) = 2*lhl*l(2,i);
  }
  return e;
}

BLA::Matrix<1,6, double> get_angles (BLA::Matrix<3,6,double> leg_leng, BLA::Matrix<1,6,double> e, BLA::Matrix<1,6, double> gg) {
  
  // calculate angle values using leg lengths and auxillary variables e and g, the mathematics can be found in the paper linked in the refrences section.
  
  BLA::Matrix<1,6, double> angles = {0,0,0,0,0,0};
  for (int i=0; i < 6; i++){
    double fk = 2* lhl* (cos(beta(0,i))* leg_leng(0,i) + sin(beta(0,i))*leg_leng(1,i));
    double nn = atan2(fk, e(0,i));
    double e_sq = e(0,i)*e(0,i);
    double fk_sq = fk*fk;
    double hh = asin(gg(0,i)/sqrt(e_sq + fk_sq));
    angles(0,i) = hh - nn;
  }
  return angles;
}

int angleToPulse(int ang)                             //gets angle in degree and returns the pulse width
  {  
     int pulse = map(ang,0, 180, SERVOMIN,SERVOMAX);  // map angle of 0 to 180 to Servo min and Servo max 
     // Serial.print("Angle: ");Serial.print(ang);
     // Serial.print(" pulse: ");Serial.println(pulse);
     return pulse;
  }
