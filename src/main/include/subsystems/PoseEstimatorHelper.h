// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/geometry/Pose2d.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/Timer.h>
#include "Constants/SwerveDriveConstants.h"
#include "Constants/SwerveModuleConstants.h"

class PoseEstimatorHelper : public frc2::SubsystemBase {
 public:
  PoseEstimatorHelper();
  void Periodic() override;
  void SetPoseEstimator(frc::SwerveDrivePoseEstimator<4> *poseEstimator); 
  void UpdatePoseEstimator(wpi::array<frc::SwerveModulePosition, 4U> modulePositions, frc::Rotation2d rotation);
  void SetActivePath(std::vector<frc::Pose2d> poses);
  void SetTargetAutoPose(frc::Pose2d pose);
  void SetCurrentAutoPose(frc::Pose2d pose);
  frc::Pose2d GetEstimatedPose();
  void UpdateRotation(frc::Rotation2d rotation);
  void AddVisionMeasurement(frc::Pose2d pose, units::second_t timestamp, wpi::array<double, 3> visionMeasurementStdDevs);
  void ResetPose(frc::Rotation2d rotation, wpi::array<frc::SwerveModulePosition, 4> modulePositions, frc::Pose2d pose);

  private:
  frc::SwerveDrivePoseEstimator<4> *m_poseEstimator;
  frc::Field2d m_estimatedPose;
  frc::Field2d m_visionPose;
  frc::Field2d m_autoField;
  frc::Rotation2d m_rotation;
};
