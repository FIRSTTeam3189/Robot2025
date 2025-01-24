// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/signals/SpnEnums.hpp>
#include <ctre/phoenix6/StatusSignal.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/core/CoreTalonFX.hpp>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/Preferences.h>
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <math.h>
#include <iostream>
#include <string>
#include <frc/geometry/Translation2d.h>
#include "subsystems/PoseEstimatorHelper.h"
#include "util/SwerveModule.h"
#include <ctre/phoenix6/Pigeon2.hpp>
#include "Constants/VisionConstants.h"
#include "Constants/AutoConstants.h"

enum class DriveState { HeadingControl, ArbitraryAngleAlign, CoralStationAlign } ;

struct SwerveModules {
  frc::Translation2d m_frontLeftLocation;
  frc::Translation2d m_frontRightLocation;
  frc::Translation2d m_backLeftLocation;
  frc::Translation2d m_backRightLocation;

  SwerveModule m_frontLeft;
  SwerveModule m_frontRight;
  SwerveModule m_backLeft;
  SwerveModule m_backRight;
};

class SwerveDrive : public frc2::SubsystemBase {
 public:
  SwerveDrive(PoseEstimatorHelper *helper);
  void ConfigGyro();
  
  void Stop();
  void Drive(units::meters_per_second_t xSpeed,
                        units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
                        bool fieldRelative,
                        frc::Translation2d centerOfRotation);
  void UpdateEstimator();
  void DriveRobotRelative(frc::ChassisSpeeds speeds);
  void SetModuleStates(std::array<frc::SwerveModuleState, 4> desiredStates);
  units::degree_t GetNormalizedYaw();
  void RefreshAllSignals();
  void ConfigSignals();
  void SetBrakeMode(BrakeMode mode);
  void SetPose(frc::Pose2d pose, bool justRotation);
  void SetSlowMode(bool slow);
  frc::Pose2d GetEstimatedPose();
  frc::Pose2d GetEstimatedAutoPose();
  frc::ChassisSpeeds GetRobotRelativeSpeeds();
  void LogModuleStates(wpi::array<frc::SwerveModulePosition, 4> modulePositions);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  SwerveModules m_modules;
  wpi::array<SwerveModule*, 4> m_moduleArray;
  PoseEstimatorHelper *m_poseHelper;
  ctre::phoenix6::hardware::Pigeon2 m_pigeon;
  wpi::array<frc::SwerveModulePosition, 4> m_modulePositions;
  std::string m_drivePKey;
  std::string m_driveIKey;
  std::string m_driveDKey;
  std::string m_anglePKey;
  std::string m_angleIKey;
  std::string m_angleDKey;
  std::string m_rotationSKey;

  double m_rotationS = SwerveDriveConstants::kSRot;

  std::vector<ctre::phoenix6::BaseStatusSignal*> m_allSignals;

  ctre::phoenix6::configs::Pigeon2Configuration m_pigeonConfigs{};

  std::string_view m_tuningModeKey = "Tuning Mode";
  std::string_view m_diagnosticsKey = "Full Diagnostics";
  bool m_slowMode = false;

  // nominalVoltage, stallTorque, stallCurrent, freeCurrent, freeSpeed, numMotors
  // frc::DCMotor m_driveMotorConfig = frc::DCMotor(
  //   AutoConstants::kNominalVoltage, AutoConstants::kStallTorque, AutoConstants::kStallCurrent, AutoConstants::kFreeCurrent, AutoConstants::kFreeSpeed, 1
  // );

  frc::DCMotor m_driveMotorConfig = frc::DCMotor(
    12.0_V, units::newton_meter_t{43.32}, units::ampere_t{366}, AutoConstants::kFreeCurrent, units::radians_per_second_t{102.8}, 1
  );

  // Wheel radius, maxDriveVelocityMPS, wheelCOF, driveMotor, driveCurrentLimit, numMotors
  pathplanner::ModuleConfig m_autoModuleConfig = pathplanner::ModuleConfig(
    units::meter_t{SwerveModuleConstants::kWheelRadiusMeters},
    SwerveModuleConstants::kMaxSpeed, 
    SwerveModuleConstants::kWheelCOF,
    // AutoConstants::kDriveMotorConfig,
    m_driveMotorConfig,
    SwerveModuleConstants::kDrivePeakCurrentLimit,
    1
  );

  // units::kilogram_t mass, units::kilogram_square_meter_t MOI, ModuleConfig moduleConfig, units::meter_t trackwidth, units::meter_t wheelbase
  pathplanner::RobotConfig m_autoRobotConfig = pathplanner::RobotConfig(
    54.431_kg, // TODO robot total mass
    units::kilogram_square_meter_t{6.750}, // TODO estimate robot as cube or smth
    m_autoModuleConfig,
    SwerveDriveConstants::kTrackwidth
    // SwerveDriveConstants::kWheelbase,
  );
};
