#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979
#endif

#define deg_to_rad(deg) (((deg)/360)*2*M_PI)
#define rad_to_deg(rad) (((rad)/2/M_PI)*360)

//Ports
    //Talons
static int kFrontRightChannel = 0, kFrontLeftChannel = 1, kRearLeftChannel = 2, kRearRightChannel = 3;
static int kElevatorMotor = 5, kElevatorMotor2 = 6, kElevatorMotor3 = 4;
static int kJoystickChannel_1 = 0, kJoystickChannel_2 = 1;
    //Sparks
static int Arm_1 = 0, Arm_2 = 1;
static int kBallGripper_top = 2, kBallGripper_bottom = 3;
static int kHatchGripper = 4;

//Values
double elevator_speed = 0.8;
double ball_gripper_speed_reduction = 1.;
double third_level_speed = 1.5;
double chassis_speed = 1.;
double calibration_threshold = 0.2;
double arm_speed_reduction = 0.6;
double last_arm_speed;

bool drive_elevator = 0;
bool hatch_gripper_state = false;

double calibration(double value){  //Function for normalizing Joystick values sent to motors
  if(-calibration_threshold<=value && value<=calibration_threshold) return 0.;
  else return value;
}

double calibrated_speed(double speed, double last_speed){
    if(abs(speed - last_speed) >= 0.1){
        if(speed > last_speed){
            last_speed += 0.1;
            return last_speed;
        }
        else {
            last_speed -= 0.1;
            return last_speed;
        }
    }
    else{
        last_speed = speed;
        return last_speed;
    }
}