#include "Robot.h"
#include "Constants.h"
#include "ctre/Phoenix.h"
#include <frc/XboxController.h>
#include <frc/Spark.h>
#include "frc/WPILib.h"
#include <frc/DigitalInput.h>
#include <frc/smartdashboard/SmartDashboard.h>

/*---------------------------------------------------------------------------------------------------------------------------------------*/
//Gripper Motors
frc::Spark arm_1(Arm_1);
frc::Spark arm_2(Arm_2);
frc::Spark ball_gripper_top(kBallGripper_top);
frc::Spark ball_gripper_bottom(kBallGripper_bottom);
frc::Spark hatch_gripper(kHatchGripper);

//Elevator Motors
WPI_TalonSRX m_elev1(kElevatorMotor);
WPI_TalonSRX m_elev2(kElevatorMotor2);
WPI_TalonSRX m_elev3(kElevatorMotor3);

//Mecanum Motors
WPI_TalonSRX m_frontLeft(kFrontLeftChannel);
WPI_TalonSRX m_rearLeft(kRearLeftChannel);
WPI_TalonSRX m_frontRight(kFrontRightChannel);
WPI_TalonSRX m_rearRight(kRearRightChannel);


//Drive
frc::MecanumDrive mecanumdrive(m_frontLeft, m_rearLeft, m_frontRight, m_rearRight);

//Xbox Controller
frc::XboxController xbox_chassis(kJoystickChannel_1);
frc::XboxController xbox_arm(kJoystickChannel_2);

//Camera
cs::UsbCamera camera;
cs::VideoSink server;

//Digital IO
frc::DigitalInput hatch_top(0);
frc::DigitalInput hatch_bottom(1);


/*---------------------------------------------------------------------------------------------------------------------------------------*/
/**
void MoveChassis(double m_fl, double m_fr, double m_bl, double m_br){
  m_frontLeft.Set(ControlMode::PercentOutput, m_fl);
  m_frontRight.Set(ControlMode::PercentOutput, m_fr);
  m_rearLeft.Set(ControlMode::PercentOutput, m_bl);
  m_rearRight.Set(ControlMode::PercentOutput, m_br);
}
void CustomMecanumDrive(double x, double y, double rightX){
  //x = calibration(x); y = calibration(y);
  double r = hypot(x, y);
  double robotAngle = atan2(y, x) - M_PI / 4;
  double v1 = r * cos(robotAngle) + rightX;
  double v2 = r * sin(robotAngle) - rightX;
  double v3 = r * sin(robotAngle) + rightX;
  double v4 = r * cos(robotAngle) - rightX;

  MoveChassis(v1, v2, v3, v4);
}
**/

void PrintValues(){
    frc::SmartDashboard::PutNumber("Left X",calibration(xbox_chassis.GetX(frc::XboxController::kLeftHand)));
    frc::SmartDashboard::PutNumber("Left Y",calibration(xbox_chassis.GetY(frc::XboxController::kLeftHand)));
    frc::SmartDashboard::PutNumber("Right X",calibration(xbox_chassis.GetX(frc::XboxController::kRightHand)));
    frc::SmartDashboard::PutNumber("Right Y",calibration(xbox_chassis.GetY(frc::XboxController::kRightHand)));
    frc::SmartDashboard::PutNumber("Left Trigger",calibration(xbox_chassis.GetTriggerAxis(frc::XboxController::kLeftHand)));
    frc::SmartDashboard::PutNumber("Right Trigger",calibration(xbox_chassis.GetTriggerAxis(frc::XboxController::kRightHand)));
}

void DriveElevator(bool left, bool right){
  if(left){
    if(drive_elevator){
      m_elev1.Set(ControlMode::PercentOutput, elevator_speed);
      m_elev2.Set(ControlMode::PercentOutput, elevator_speed);
    }
    m_elev3.Set(ControlMode::PercentOutput, elevator_speed);
  }
  else if(right){
    if(drive_elevator){
      m_elev1.Set(ControlMode::PercentOutput, -elevator_speed);
      m_elev2.Set(ControlMode::PercentOutput, -elevator_speed);
    }
    m_elev3.Set(ControlMode::PercentOutput, -elevator_speed);
  }
  else{
    m_elev1.SetNeutralMode(NeutralMode::Brake);
    m_elev2.SetNeutralMode(NeutralMode::Brake);
    m_elev3.SetNeutralMode(NeutralMode::Brake);
    m_elev1.Set(ControlMode::PercentOutput, 0.);
    m_elev2.Set(ControlMode::PercentOutput, 0.);
    m_elev3.Set(ControlMode::PercentOutput, 0.);
  }
}

//正負を要確認！！！
void DriveGripperArm(double joystick_value){
  if((!hatch_top.Get() && -joystick_value < 0.) || !hatch_bottom.Get() && -joystick_value > 0.){
    arm_1.Set(-joystick_value * arm_speed_reduction);
    arm_2.Set(-joystick_value * arm_speed_reduction);
  }
  else{
    arm_1.StopMotor();
    arm_2.StopMotor();
  }
}

void DriveBallGripper(double joystick_value){
  if(joystick_value != 0.){
    ball_gripper_top.Set(joystick_value * ball_gripper_speed_reduction);
    ball_gripper_bottom.Set(joystick_value * ball_gripper_speed_reduction);
  }
  else{
    ball_gripper_top.StopMotor();
    ball_gripper_bottom.StopMotor();
  }
}

void DriveHatchGripper(bool button){
  if(button){
    if(hatch_gripper_state == false) hatch_gripper_state = true;
    else hatch_gripper_state = false;
  }
  if(hatch_gripper_state == true)hatch_gripper.Set(1.);
  else hatch_gripper.Set(-1.);
}

/*---------------------------------------------------------------------------------------------------------------------------------------*/
void Robot::RobotInit() {
  //Camera Setup
  camera = frc::CameraServer::GetInstance()->StartAutomaticCapture(0);
  server = frc::CameraServer::GetInstance()->GetServer();
  camera.SetConnectionStrategy(cs::VideoSource::ConnectionStrategy::kConnectionKeepOpen);

  //hatch_top = new frc::DigitalInput(0);
  //hatch_bottom = new frc::DigitalInput(1);

}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {
}

void Robot::AutonomousPeriodic() {
}

void Robot::TeleopInit() {
  arm_1.EnableDeadbandElimination(true);
  arm_2.EnableDeadbandElimination(true);
  ball_gripper_top.EnableDeadbandElimination(true);
  ball_gripper_bottom.EnableDeadbandElimination(true);
  hatch_gripper.EnableDeadbandElimination(true);
}

void Robot::TeleopPeriodic() {
  server.SetSource(camera);
  //Chassis
  //CustomMecanumDrive(calibration(xbox_chassis.GetX(frc::XboxController::kLeftHand)), calibration(xbox_chassis.GetY(frc::XboxController::kLeftHand)), calibration(xbox_chassis.GetX(frc::XboxController::kRightHand)));
  mecanumdrive.DriveCartesian(calibration(xbox_chassis.GetX(frc::XboxController::kLeftHand)), -calibration(xbox_chassis.GetY(frc::XboxController::kLeftHand)), calibration(xbox_chassis.GetX(frc::XboxController::kRightHand)));
  
  //Elevator
  DriveElevator(xbox_chassis.GetBumper(frc::XboxController::kLeftHand), xbox_chassis.GetBumper(frc::XboxController::kRightHand));

  //Gripper
  DriveGripperArm(calibration(xbox_arm.GetY(frc::XboxController::kLeftHand)));
  DriveBallGripper(calibration(xbox_arm.GetY(frc::XboxController::kRightHand)));
  DriveHatchGripper(xbox_arm.GetAButton());

  PrintValues();
}

void Robot::TestPeriodic() {
  PrintValues();
}

/*---------------------------------------------------------------------------------------------------------------------------------------*/

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif