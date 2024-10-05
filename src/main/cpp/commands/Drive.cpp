// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Drive.h"

Drive::Drive(frc2::CommandJoystick *joystick, SwerveDrive *swerveDrive, DriveState driveState, units::degree_t arbitraryAngle) :
m_bill(joystick),
m_swerveDrive(swerveDrive),
m_swerveAlignUtil(swerveDrive),
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
