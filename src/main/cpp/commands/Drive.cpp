// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/Preferences.h>
#include "commands/Drive.h"
#include "Constants/OperatorConstants.h"

Drive::Drive(frc::PS5Controller *joystick, SwerveDrive *swerveDrive, DriveState driveState, CoralStationTarget coralStationTarget, units::degree_t arbitraryAngle) :
m_bill(joystick),
m_swerveDrive(swerveDrive),
m_rotationPIDController(SwerveDriveConstants::kPRot, SwerveDriveConstants::kIRot, SwerveDriveConstants::kDRot),
m_driveState(driveState),
m_arbitraryAngle(arbitraryAngle),
m_coralStationTarget(coralStationTarget),
m_allianceSide(frc::DriverStation::Alliance::kBlue) {
  (void)VisionConstants::kSyncBytes[0];

  AddRequirements(swerveDrive);

  if (frc::DriverStation::GetAlliance())
    m_allianceSide = frc::DriverStation::GetAlliance();

  // Add tolerance value of 1
  m_rotationPIDController.SetTolerance(SwerveDriveConstants::kSwerveRotationTolerance);
  m_rotationPIDController.EnableContinuousInput(-180, 180);

  m_rotationPKey = "Robot Rotation P";
  m_rotationIKey = "Robot Rotation I";
  m_rotationDKey = "Robot Rotation D";

  frc::Preferences::SetDouble(m_rotationPKey, SwerveDriveConstants::kPRot);
  frc::Preferences::SetDouble(m_rotationIKey, SwerveDriveConstants::kIRot);
  frc::Preferences::SetDouble(m_rotationDKey, SwerveDriveConstants::kDRot);
}

void Drive::Execute(){
  UpdatePreferences();

  double joystickX = 0.0, joystickY = 0.0;
  m_allianceSide = frc::DriverStation::GetAlliance();

   if (m_allianceSide) {
    if (m_allianceSide.value() == frc::DriverStation::Alliance::kBlue) {
      joystickX = -m_bill->GetRawAxis(OperatorConstants::kAxisLeftStickY);
      joystickY = -m_bill->GetRawAxis(OperatorConstants::kAxisLeftStickX);
    } else {
      joystickX = m_bill->GetRawAxis(OperatorConstants::kAxisLeftStickY);
      joystickY = m_bill->GetRawAxis(OperatorConstants::kAxisLeftStickX);
    }
  }

  joystickX = (0.4 * joystickX) + (0.6 * pow(joystickX, 3));
  joystickY = (0.4 * joystickY) + (0.6 * pow(joystickY, 3));

  units::meters_per_second_t xSpeed = (m_xSpeedLimiter.Calculate(joystickX) * SwerveDriveConstants::kMaxSpeed);
  units::meters_per_second_t ySpeed = (m_ySpeedLimiter.Calculate(joystickY) * SwerveDriveConstants::kMaxSpeed);

  if (fabs(joystickX) < .05)
    xSpeed = 0_mps;
  if (fabs(joystickY) < .05)
    ySpeed = 0_mps;

  frc::SmartDashboard::PutNumber("X speed", xSpeed.value());
  frc::SmartDashboard::PutNumber("Y speed", ySpeed.value());
  units::radians_per_second_t rot;

   // For now, just heading control, but added switch statement in case of new additions
   switch (m_driveState) {
    case(DriveState::HeadingControl) :
      rot = GetDesiredRotationalVelocity();
      break;
    case(DriveState::ArbitraryAngleAlign) :
      rot = units::angular_velocity::radians_per_second_t{
              m_rotLimiter.Calculate(m_rotationPIDController.Calculate(m_swerveDrive->GetNormalizedYaw().value(), m_arbitraryAngle.value()))
              * SwerveDriveConstants::kMaxAngularVelocity};
      break;
     case(DriveState::CoralStationAlign) :
      {
        // units::degree_t targetAngle = GetSourceAlignAngle(m_coralStationTarget);
        units::degree_t targetAngle = GetSourceAlignAngleAutomatically();
        rot = units::angular_velocity::radians_per_second_t{
              m_rotLimiter.Calculate(m_rotationPIDController.Calculate(m_swerveDrive->GetNormalizedYaw().value(), targetAngle.value()))
              * SwerveDriveConstants::kMaxAngularVelocity};
        break;
      }
    default:
      rot = units::angular_velocity::radians_per_second_t{0.0};
      break;
  }

  frc::SmartDashboard::PutNumber("Robot desired rotation (rad per s)", rot.value());
  m_swerveDrive->Drive(xSpeed, ySpeed, rot, true, frc::Translation2d{});
}

units::angular_velocity::radians_per_second_t Drive::GetDesiredRotationalVelocity() { 
   // Get raw (-1.0 to 1.0) joystick positions for x and y axis
  // Left, up are -1.0; right, down are 1.0
  // Inverted so forward on joystick is down the field
  // If red alliance, flip 180
  m_allianceSide = frc::DriverStation::GetAlliance();
  double joystickX = 0.0, joystickY = 0.0;
  
  if (m_allianceSide) {
    if (m_allianceSide.value() == frc::DriverStation::Alliance::kRed) {
      frc::SmartDashboard::PutString("Alliance", "red");
      joystickX = m_bill->GetRightY();
      joystickY = m_bill->GetRightX();
    } else {
      frc::SmartDashboard::PutString("Alliance", "blue");
      joystickX = -(m_bill->GetRightY());
      joystickY = -(m_bill->GetRightX());
    }
  }

  // Manual deadband to inputs greater than 5% only
  // If deadband detected (i.e. user is not giving rotation input), then set goalAngle as last desired angle
  if ((fabs(joystickX) < .05) && (fabs(joystickY) < .05)) {
    // m_goalAngle = m_lastAngle;
    return units::angular_velocity::radians_per_second_t{0.0};
  } else {
    // Convert joystick positions to goal angle in degrees
    // Normalized from -180, 180
    // Uses arctan2 function -- converts Cartesian coordinates (1, 1) to a polar angle (pi / 4), then multiplies by radians to degrees conversion
    // Converts rad to degrees
    m_goalAngle = SwerveDriveConstants::kRadiansToDegreesMultiplier * atan2(joystickY, joystickX);
  }
  m_lastAngle = m_goalAngle;

  frc::SmartDashboard::PutNumber("Robot Desired Angle", m_goalAngle);

  // Return next velocity in radians per second as calculated by PIDController and limited by rotLimiter
  units::angular_velocity::radians_per_second_t rot = 
              units::angular_velocity::radians_per_second_t{
              m_rotLimiter.Calculate(m_rotationPIDController.Calculate(m_swerveDrive->GetNormalizedYaw().value(), m_goalAngle))
              * SwerveDriveConstants::kMaxAngularVelocity};

  // converts a given or desired coordinate to a degree on the unit circle

  frc::SmartDashboard::PutNumber("Rotation PID Output (deg per s)", rot.value() * (180.0 / PI));

  if (abs(m_swerveDrive->GetNormalizedYaw().value() - m_goalAngle) < SwerveDriveConstants::kSwerveRotationTolerance) {
    // Clear integral sum
    frc::SmartDashboard::PutBoolean("Integral gain reset", true);
    m_rotationPIDController.Reset();
    rot = units::angular_velocity::radians_per_second_t{0.0};
  }

  return rot;
}

units::angle::degree_t Drive::GetSourceAlignAngleAutomatically(){
  auto currentPose = m_swerveDrive->GetEstimatedPose();
  bool isBlueAlliance;
  if(frc::DriverStation::GetAlliance()){
    auto isBlueAlliance = frc::DriverStation::GetAlliance().value() == frc::DriverStation::Alliance::kBlue;
  }
  return (currentPose.Y().value() >= SwerveDriveConstants::kMidFieldY) 
        ? (isBlueAlliance ? SwerveDriveConstants::kBlueSourceAlignTargetTop : SwerveDriveConstants::kRedSourceAlignTargetTop)
        : (isBlueAlliance ? SwerveDriveConstants::kBlueSourceAlignTargetBottom : SwerveDriveConstants::kRedSourceAlignTargetRightBottom);
}

units::angle::degree_t Drive::GetSourceAlignAngle(CoralStationTarget target){
  auto angle = 0.0_deg;
   switch (target){
    case(CoralStationTarget::RedTop) :
      angle = SwerveDriveConstants::kRedSourceAlignTargetTop;
      break;
    case(CoralStationTarget::RedBottom) :
      angle = SwerveDriveConstants::kRedSourceAlignTargetRightBottom;
      break;
    case(CoralStationTarget::BlueTop) :
      angle = SwerveDriveConstants::kBlueSourceAlignTargetTop;
      break;
    case(CoralStationTarget::BlueBottom) :
      angle = SwerveDriveConstants::kBlueSourceAlignTargetBottom;
      break;
    default:
      angle = 0.0_deg;
   }
    return angle;
}

void Drive::UpdatePreferences() {
  if (frc::Preferences::GetBoolean("Tuning Mode", false)) {
    m_rotationPIDController.SetP(frc::Preferences::GetDouble(m_rotationPKey, SwerveDriveConstants::kPRot));
    m_rotationPIDController.SetI(frc::Preferences::GetDouble(m_rotationIKey, SwerveDriveConstants::kIRot));
    m_rotationPIDController.SetD(frc::Preferences::GetDouble(m_rotationDKey, SwerveDriveConstants::kDRot));
  }
}

void Drive::Initialize(){}

bool Drive::IsFinished(){
  return false;
}

// Called once the command ends or is interrupted.
void Drive::End(bool interrupted) {
  m_swerveDrive->Stop();
}
