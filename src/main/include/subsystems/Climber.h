// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/length.h>

#include <frc/Preferences.h>

#include "Constants/GlobalConstants.h"
#include "Constants/CoralMechanismConstants.h"

class Climber : public frc2::SubsystemBase {
 public:
  Climber();
  void SetExtensionBrakeMode(BrakeMode mode);
  void ConfigClimberMotor();
  void ConfigPID();
  void SetPower(double voltage);
  void RefreshAllSignals();
  units::meter_t GetExtension();
  void UpdatePreferences();
  void SetExtension(units::meter_t target);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  ctre::phoenix6::hardware::TalonFX m_climberMotor;
  ctre::phoenix6::configs::TalonFXConfiguration m_climberConfig{};

  units::meter_t m_targetHeightClimber;
  ctre::phoenix6::StatusSignal<units::angle::turn_t> m_climberHeight = m_climberMotor.GetPosition();
  ctre::phoenix6::StatusSignal<units::angular_velocity::turns_per_second_t> m_climberVelocity = m_climberMotor.GetVelocity();
  std::vector<ctre::phoenix6::BaseStatusSignal*> m_allSignals;

  std::string m_climberPKey;
  std::string m_climberIKey;
  std::string m_climberDKey;
  std::string m_climberGKey;
  std::string m_climberSKey; //meow
  std::string m_climberVKey;
  std::string m_climberAKey;
  std::string m_climberTargetKey;
};
