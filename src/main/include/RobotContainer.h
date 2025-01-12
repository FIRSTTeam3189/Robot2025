// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/Command.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/button/Trigger.h>
#include <frc/PS5Controller.h>
#include <frc2/command/InstantCommand.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/WaitCommand.h>

#include "commands/Drive.h"
#include "commands/SetAlgaeIntakeRotation.h"
#include "commands/RunAlgaeIntakeRoller.h"

#include "commands/SetAlgaeIntakeRotation.h"
#include "commands/SetCoralElevatorExtension.h"
#include "commands/SetCoralManipulatorRotation.h"

#include "subsystems/PoseEstimatorHelper.h"
#include "subsystems/Vision.h"
#include "subsystems/AlgaeIntake.h"
#include "subsystems/CoralElevator.h"
#include "subsystems/CoralManipulator.h"

#include "Constants/OperatorConstants.h"
#include "Constants/AutoConstants.h"
#include "Constants/SwerveDriveConstants.h"

#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/util/PathPlannerLogging.h>

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();
  bool IsClimbState();
  BrakeMode GetBrakeMode();
  void SetAllCoast();
  void SetAllNormalBrakeMode();

 private:
  // Replace with CommandPS4Controller or CommandJoystick if needed
  frc::PS5Controller m_bill{OperatorConstants::kDriverControllerPort};
  frc::PS5Controller m_ted{OperatorConstants::kCoDriverControllerPort};
  frc::PS5Controller m_test{OperatorConstants::kTestControllerPort};

  // The robot's subsystems are defined here...
  PoseEstimatorHelper *m_poseEstimator = new PoseEstimatorHelper();
  SwerveDrive *m_swerveDrive = new SwerveDrive(m_poseEstimator);
  // Vision *m_vision = new Vision(m_poseEstimator);
  // AlgaeIntake *m_algaeIntake = new AlgaeIntake();
  // CoralElevator *m_coralElevator = new CoralElevator();
  // CoralManipulator *m_coralManipulator = new CoralManipulator();

  frc::SendableChooser<frc2::Command*> m_chooser;

  DriveState m_driveState = DriveState::HeadingControl;

  void ConfigureDriverBindings();
  void ConfigureCoDriverBindings();
  void ConfigureTestBindings();
  void CreateAutoPaths();
  void RegisterAutoCommands();
};