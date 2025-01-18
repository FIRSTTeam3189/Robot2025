#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/sysid/SysIdRoutine.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <frc/Preferences.h>

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/core/CoreTalonFX.hpp>
#include <ctre/phoenix6/StatusSignal.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>

#include "Constants/GlobalConstants.h"
#include "Constants/AlgaeIntakeConstants.h"
#include "Constants/AlgaeIntakeConstants.h"

// IntakeAlgae extends it to the target to get the algae and spin rollers, ScoreProcessor goes to vertical position and DefaultRetract is its state when doing nothing (retracted in the robot)
enum class AlgaeIntakeState { IntakeAlgae, ScoreProcessor, DefaultRetract };

class AlgaeIntake : public frc2::SubsystemBase {
 public:
  AlgaeIntake();
  void SetRollerPower(double power);
  void SetRotation(units::degree_t target);
  units::degree_t GetRotation();
  units::degree_t GetCurrentTargetAngle();
  void SetState(AlgaeIntakeState state);
  void SetRotationBrakeMode(BrakeMode mode);
  void UpdatePreferences();
  void ConfigRollerMotor();
  void ConfigRotationMotor();
  void ConfigRotationCANcoder();
  void RefreshAllSignals();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 protected:
  void ConfigPID();

 private:
  ctre::phoenix6::hardware::TalonFX m_rotationMotor;
  ctre::phoenix6::hardware::CANcoder m_CANcoder;
  rev::spark::SparkMax m_rollerMotor;

  ctre::phoenix6::configs::TalonFXConfiguration m_rotationConfig{};
  ctre::phoenix6::configs::CANcoderConfiguration m_encoderConfig{};
  rev::spark::SparkMaxConfig m_rollerConfig;

  AlgaeIntakeState m_target;
  units::degree_t m_targetAngle;
  ctre::phoenix6::StatusSignal<units::angle::turn_t> m_rotationAngle = m_rotationMotor.GetPosition();
  ctre::phoenix6::StatusSignal<units::angular_velocity::turns_per_second_t> m_rotationVelocity = m_rotationMotor.GetVelocity();
  std::vector<ctre::phoenix6::BaseStatusSignal*> m_allSignals;
  
  // String keys for PID preferences
  std::string m_rotationPKey;
  std::string m_rotationIKey;
  std::string m_rotationDKey;
  std::string m_rotationGKey;
  std::string m_rotationSKey;
  std::string m_rotationVKey;
  std::string m_rotationAKey;
  std::string m_rotationTargetKey;
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};


