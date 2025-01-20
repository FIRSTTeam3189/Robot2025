// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>

void Robot::RobotInit() {
  // Default to first USB flash drive or home/lvuser/logs/ if none available
  ctre::phoenix6::SignalLogger::SetPath("");
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *f
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();
}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {
  m_container.SetElevatorCoast();
  m_container.SetDriveCoast();
}

void Robot::DisabledPeriodic() {}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit() {
  // m_container.SetAllNormalBrakeMode();
  m_autonomousCommand = m_container.GetAutonomousCommand();

  frc::SmartDashboard::PutBoolean("Auto command interrupted", false);

  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {
  frc::SmartDashboard::PutData("Command scheduler", &frc2::CommandScheduler::GetInstance());

  m_container.PrintActiveCommands();
  
  frc::SmartDashboard::PutBoolean("Auto command/is finished", m_autonomousCommand->IsFinished()); 
}

void Robot::TeleopInit() {
  m_container.SetAllBrake();
   if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Cancel();
  }
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {
  m_container.PrintActiveCommands();
}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

/**
 * This function is called once when the robot is first started up.
 */
void Robot::SimulationInit() {}

/**
 * This function is called periodically whilst in simulation.
 */
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  (void)VisionConstants::kSyncBytes[0];
  return frc::StartRobot<Robot>();
}
#endif
