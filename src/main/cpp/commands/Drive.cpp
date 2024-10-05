// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/Preferences.h>
#include "commands/Drive.h"
#include "Constants/OperatorConstants.h"

Drive::Drive(frc2::CommandJoystick *joystick, SwerveDrive *swerveDrive, DriveState driveState, units::degree_t arbitraryAngle) :
m_bill(joystick),
m_swerveDrive(swerveDrive),
m_rotationPIDController(SwerveDriveConstants::kPRot, SwerveDriveConstants::kIRot, SwerveDriveConstants::kDRot),
m_driveState(driveState),
m_arbitraryAngle(arbitraryAngle),
m_allianceSide(frc::DriverStation::Alliance::kBlue) {
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
      
    default:
      rot = units::angular_velocity::radians_per_second_t{0.0};
      break;
  }

  frc::SmartDashboard::PutNumber("Robot desired rotation (rad per s)", rot.value());
  m_swerveDrive->Drive(xSpeed, ySpeed, rot, true, frc::Translation2d{});


units::angular_velocity::radians_per_second_t GetDesiredRotationalVelocity(){
  // TODO
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
