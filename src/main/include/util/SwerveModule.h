#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/signals/SpnEnums.hpp>
#include <ctre/phoenix6/StatusSignal.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/core/CoreTalonFX.hpp>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/Preferences.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <math.h>
#include <iostream>
#include <string>
#include <Constants/GlobalConstants.h>

class SwerveModule {
 public:
  SwerveModule(int moduleNumber, int driveMotorID, int angleMotorID,
               int CANcoderID, double CANcoderOffset);
  void ConfigDriveMotor();
  void ConfigAngleMotor(int CANcoderID);
  void ConfigCANcoder();
  void SetBrakeMode(BrakeMode mode);
  void Stop();
  void UpdatePosition();
  void SetDesiredState(const frc::SwerveModuleState &state);

}


