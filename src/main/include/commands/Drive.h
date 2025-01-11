#pragma once

#include <frc/filter/SlewRateLimiter.h>
#include <frc/DriverStation.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/controller/PIDController.h>
#include <frc/Joystick.h>
#include <frc/PS5Controller.h>
#include "subsystems/SwerveDrive.h"

enum class CoralStationTarget { RedTop, RedBottom, BlueTop, BlueBottom };

class Drive
    : public frc2::CommandHelper<frc2::Command, Drive> {
 public:
  Drive(frc::PS5Controller *joystick, SwerveDrive *swerveDrive, DriveState driveState, CoralStationTarget coralStationTarget = CoralStationTarget::RedTop, units::degree_t arbitraryAngle = 0.0_deg);
  units::angular_velocity::radians_per_second_t GetDesiredRotationalVelocity();

  // For PID tuning
  void UpdatePreferences();

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  units::degree_t GetSourceAlignAngle(CoralStationTarget targetStation);

 private:
  frc::PS5Controller *m_bill;
  SwerveDrive *m_swerveDrive;
  frc::PIDController m_rotationPIDController;
  DriveState m_driveState;
  units::degree_t m_arbitraryAngle;
  CoralStationTarget m_coralStationTarget;
  frc::SlewRateLimiter<units::scalar> m_xSpeedLimiter{2 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_ySpeedLimiter{2 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_rotLimiter{2 / 1_s};
  double m_goalAngle;
  double m_lastAngle;
  std::optional<frc::DriverStation::Alliance> m_allianceSide;
  std::string m_rotationPKey;
  std::string m_rotationIKey;
  std::string m_rotationDKey;
};