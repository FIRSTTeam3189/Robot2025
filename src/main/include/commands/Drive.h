#pragma once

#include <frc/filter/SlewRateLimiter.h>
#include <frc/DriverStation.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/controller/PIDController.h>
#include <frc/Joystick.h>
#include <frc2/command/button/CommandJoystick.h>
#include "subsystems/SwerveDrive.h"

class Drive
    : public frc2::CommandHelper<frc2::Command, Drive> {
 public:
  Drive(frc2::CommandJoystick *joystick, SwerveDrive *swerveDrive, DriveState driveState, units::degree_t arbitraryAngle = 0.0_deg);
  units::angular_velocity::radians_per_second_t GetDesiredRotationalVelocity();

  // For PID tuning
  void UpdatePreferences();

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  frc2::CommandJoystick *m_bill;
  SwerveDrive *m_swerveDrive;
  frc::PIDController m_rotationPIDController;
  DriveState m_driveState;
  units::degree_t m_arbitraryAngle;
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