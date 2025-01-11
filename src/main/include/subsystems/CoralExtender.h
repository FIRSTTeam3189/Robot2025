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

enum class CoralExtenderState { DefaultRetract, L1, L2, L3, L4, Intake };

class CoralExtender : public frc2::SubsystemBase {
 public:
  CoralExtender();
  units::meter_t GetExtension();
  units::meter_t GetCurrentTargetHeight();
  void SetState(CoralExtenderState state);
  void SetExtension(units::meter_t target);
  void SetExtensionBrakeMode(BrakeMode mode);
  void UpdatePreferences();
  void ConfigExtensionMotor();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 protected:
  void ConfigPID();

 private:
  ctre::phoenix6::hardware::TalonFX m_extensionMotor;
  ctre::phoenix6::configs::TalonFXConfiguration m_extensionConfig{};

  CoralExtenderState m_state;
  units::meter_t m_targetHeight;
  ctre::phoenix6::StatusSignal<units::angle::turn_t> m_extensionHeight = m_extensionMotor.GetPosition();
  ctre::phoenix6::StatusSignal<units::angular_velocity::turns_per_second_t> m_extensionVelocity = m_extensionMotor.GetVelocity();
  
  // String keys for PID preferences
  std::string m_extensionPKey;
  std::string m_extensionIKey;
  std::string m_extensionDKey;
  std::string m_extensionGKey;
  std::string m_extensionSKey;
  std::string m_extensionVKey;
  std::string m_extensionAKey;
  std::string m_extensionTargetKey;
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
