// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>

RobotContainer::RobotContainer() {
  (void)VisionConstants::kSyncBytes[0];
  (void)AutoConstants::kAutonomousPaths[0];
  RegisterAutoCommands();
  
  // Initialize all of your commands and subsystems here 

  // Grant default control of swerve drive to either bill (controller 0),
  // or test (controller 2), based on a constant
  switch (SwerveDriveConstants::kActiveController) {
    case(ActiveDriveController::OfficialDriver) :
      m_swerveDrive->SetDefaultCommand(Drive(&m_bill, m_swerveDrive, m_driveState));
      break;
    case(ActiveDriveController::TestControls) :
      m_swerveDrive->SetDefaultCommand(Drive(&m_test, m_swerveDrive, m_driveState));
      break;
    default :
      break;
  }

  frc::SmartDashboard::PutData("Auto Routines", &m_chooser);
  
  // Configure the button bindings
  ConfigureDriverBindings();
  ConfigureCoDriverBindings();
  ConfigureTestBindings();
  // CreateAutoPaths();
}

void RobotContainer::ConfigureDriverBindings() {
  // Bill controls
  // reset the pose of the robot in the case of noise or natural dampening
  frc2::Trigger resetPoseButton([this](){ return m_bill.GetTouchpadButton(); });
  resetPoseButton.OnTrue(frc2::InstantCommand([this]{
    if (frc::DriverStation::GetAlliance()) {
      if (frc::DriverStation::GetAlliance().value() == frc::DriverStation::Alliance::kBlue)
        m_swerveDrive->SetPose(SwerveDriveConstants::kBlueResetPose, false);
      else
        m_swerveDrive->SetPose(SwerveDriveConstants::kRedResetPose, false);
    }
  },{m_swerveDrive}).ToPtr());
}

void RobotContainer::ConfigureCoDriverBindings() {
  // Ted controls
}

void RobotContainer::RegisterAutoCommands() {
  // Start of Auto Events
  pathplanner::NamedCommands::registerCommand("PrintAutoMessage", frc2::InstantCommand([this]{
    for (int i = 0; i < 10; i++) {
      std::cout << "Auto started/ended\n"; }},{}).ToPtr());
} 

void RobotContainer::CreateAutoPaths() {
  for (auto autoPath : AutoConstants::kAutonomousPaths) {
    m_chooser.AddOption(autoPath, new pathplanner::PathPlannerAuto(std::string{autoPath}));
    for (int i = 0; i < 10; i++) {
      std::cout << "Constructing path:" << std::string{autoPath} << std::endl;
    }
    std::cout << "\n";
  }
  frc::SmartDashboard::PutData("Auto Routines", &m_chooser);

  // Logging callbacks for pathplanner -- current pose, target pose, and active path
  pathplanner::PathPlannerLogging::setLogCurrentPoseCallback([this](frc::Pose2d pose) {
    // Do whatever you want with the poses here
    m_poseEstimator->SetCurrentAutoPose(pose);
  });

  pathplanner::PathPlannerLogging::setLogTargetPoseCallback([this](frc::Pose2d pose) {
    // Do whatever you want with the poses here
    m_poseEstimator->SetTargetAutoPose(pose);
  });

  // Logging callback for the active path, this is sent as a vector of poses
  pathplanner::PathPlannerLogging::setLogActivePathCallback([this](std::vector<frc::Pose2d> poses) {
    // Do whatever you want with the poses here
    m_poseEstimator->SetActivePath(poses);
  });
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return m_chooser.GetSelected();
}

void RobotContainer::SetAllCoast() {
  m_swerveDrive->SetBrakeMode(BrakeMode::Coast);
}

// motors will coast along

void RobotContainer::SetAllNormalBrakeMode() {
  m_swerveDrive->SetBrakeMode(BrakeMode::Default);
}

void RobotContainer::ConfigureTestBindings() {
  frc2::Trigger resetPoseButton([this](){ return m_test.GetTouchpadButton(); });
  resetPoseButton.OnTrue(frc2::InstantCommand([this]{
    if (frc::DriverStation::GetAlliance()) {
      if (frc::DriverStation::GetAlliance().value() == frc::DriverStation::Alliance::kBlue)
        m_swerveDrive->SetPose(SwerveDriveConstants::kBlueResetPose, false);
      else
        m_swerveDrive->SetPose(SwerveDriveConstants::kRedResetPose, false);
    }
  },{m_swerveDrive}).ToPtr());

  frc2::Trigger coralStationAlignButton([this](){ return m_test.GetL2Button(); });
  coralStationAlignButton.OnTrue(frc2::InstantCommand([this]{
    // if (m_driveState == DriveState::HeadingControl) {
      m_driveState = DriveState::CoralStationAlign;
    // } else {
    //   m_driveState = DriveState::HeadingControl;
    // }
      m_swerveDrive->SetDefaultCommand(Drive(&m_test, m_swerveDrive, m_driveState));
    },{m_swerveDrive}).ToPtr()
  );
  coralStationAlignButton.OnFalse(frc2::InstantCommand([this]{
      m_driveState = DriveState::HeadingControl;
      m_swerveDrive->SetDefaultCommand(Drive(&m_test, m_swerveDrive, m_driveState));
    },{m_swerveDrive}).ToPtr()
  );

  // frc2::Trigger intakeCoralButton([this](){ return m_test.GetL2Button(); });
  // intakeCoralButton.OnTrue(frc2::ParallelCommandGroup(
  //     SetCoralElevatorExtension(m_coralElevator, CoralElevatorState::Intake),
  //     SetCoralManipulatorRotation(m_coralManipulator, CoralManipulatorTarget::Intake)
  //   ).ToPtr()
  // );
  // intakeCoralButton.OnFalse(frc2::ParallelCommandGroup(
  //     SetCoralElevatorExtension(m_coralElevator, CoralElevatorState::DefaultRetract),
  //     SetCoralManipulatorRotation(m_coralManipulator, CoralManipulatorTarget::DefaultPosition)
  //   ).ToPtr()
  // );

  // TODO
  // frc2::Trigger scoreCoralButton([this](){ return m_test.GetR2Button(); });
  // scoreCoralButton.OnTrue(SetCoralManipulatorRotation(m_coralManipulator, CoralManipulatorTarget::ScoreCoralL123).ToPtr());
  // scoreCoralButton.OnFalse(frc2::ParallelCommandGroup(
  //     SetCoralElevatorExtension(m_coralElevator, CoralElevatorState::DefaultRetract),
  //     SetCoralManipulatorRotation(m_coralManipulator, CoralManipulatorTarget::DefaultPosition)
  //   ).ToPtr()
  // );

  // // 5 buttons below just set height of elevator to desired height when pressed
  // frc2::Trigger retractCoralElevatorButton([this](){ return m_test.GetTouchpadButton(); });
  // retractCoralElevatorButton.OnTrue(
  //   SetCoralElevatorExtension(m_coralElevator, CoralElevatorState::DefaultRetract).ToPtr()
  // );

  // frc2::Trigger extendCoralElevatorL1Button([this](){ return m_test.GetCrossButton(); });
  // extendCoralElevatorL1Button.OnTrue(
  //   SetCoralElevatorExtension(m_coralElevator, CoralElevatorState::L1).ToPtr()
  // );

  // frc2::Trigger extendCoralElevatorL2Button([this](){ return m_test.GetCircleButton(); });
  // extendCoralElevatorL2Button.OnTrue(
  //   SetCoralElevatorExtension(m_coralElevator, CoralElevatorState::L2).ToPtr()
  // );

  // frc2::Trigger extendCoralElevatorL3Button([this](){ return m_test.GetTriangleButton(); });
  // extendCoralElevatorL3Button.OnTrue(
  //   SetCoralElevatorExtension(m_coralElevator, CoralElevatorState::L3).ToPtr()
  // );

  // frc2::Trigger extendCoralElevatorL4Button([this](){ return m_test.GetSquareButton(); });
  // extendCoralElevatorL4Button.OnTrue(
  //   SetCoralElevatorExtension(m_coralElevator, CoralElevatorState::L4).ToPtr()
  // );
}

// no matter how nice ethan might seem, when you least expect it he will slap you with a piece of chicken and eat you in a bucket