// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>

#include <frc/Preferences.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <frc/controller/ArmFeedforward.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>

#include <frc/Timer.h>

#include "Constants/CoralManipulatorConstants.h"

// ScoreCoralL123 just sets it to -35 degrees for the levels 1-3
// Hold position keeps previous target while GoTarget generates a motion profile
enum class CoralManipulatorState {HoldPosition, GoTarget};
enum class CoralManipulatorTarget { DefaultPosition, ScoreCoralL123, ScoreCoralL4 };

class CoralManipulator : public frc2::SubsystemBase {
 public:
  CoralManipulator();

  void ConfigRotationMotor();
  void ConfigRollerMotor();
  void ConfigPID();

  void UpdatePreferences();

  void SetState(CoralManipulatorState state, CoralManipulatorTarget target = CoralManipulatorTarget::DefaultPosition);
  void SetRotation(units::degree_t targetAngle);
  units::degree_t GetTargetAngleFromTarget(CoralManipulatorTarget target);
  units::volt_t GetMotionProfileFeedForwardValue();
  units::degree_t GetRotation();
  units::degree_t GetCurrentTargetAngle();


  void Periodic() override;
  

 private:
    rev::spark::SparkMax m_rotationMotor;
    rev::spark::SparkAbsoluteEncoder m_rotationEncoder;
    rev::spark::SparkMaxConfig m_rotationConfig;

    frc::ArmFeedforward *m_ff;
    frc::ProfiledPIDController<units::degrees> m_profiledPIDController;
    frc::TrapezoidProfile<units::degrees>::Constraints m_constraints;


    units::degree_t m_targetAngle;
    CoralManipulatorState m_state;

    std::string m_rotationPKey;
    std::string m_rotationIKey;
    std::string m_rotationDKey;
    std::string m_rotationGKey;
    std::string m_rotationSKey;
    std::string m_rotationVKey;
    std::string m_rotationAKey;
    std::string m_rotationTargetKey;

    units::degrees_per_second_t m_lastSpeed;
    units::degrees_per_second_t m_lastTargetSpeed;
    units::degrees_per_second_squared_t m_acceleration;
    units::degrees_per_second_squared_t m_targetAcceleration;
    units::second_t m_lastTime;
};
