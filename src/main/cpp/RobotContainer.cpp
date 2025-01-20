// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>

RobotContainer::RobotContainer() {
  (void)VisionConstants::kSyncBytes[0];
  RegisterAutoCommands();
  m_chooser = pathplanner::AutoBuilder::buildAutoChooser();
  
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
  CreateAutoPaths();
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
  frc2::Trigger intakeCoralButton([this](){ return m_ted.GetL2Button(); });
  intakeCoralButton.OnTrue(frc2::ParallelCommandGroup(
      SetCoralElevatorExtension(m_coralElevator, CoralElevatorState::Intake),
      SetCoralManipulatorRotation(m_coralManipulator, CoralManipulatorTarget::Intake)
    ).ToPtr()
  );
  intakeCoralButton.OnFalse(frc2::ParallelCommandGroup(
      SetCoralElevatorExtension(m_coralElevator, CoralElevatorState::DefaultRetract),
      SetCoralManipulatorRotation(m_coralManipulator, CoralManipulatorTarget::DefaultPosition)
    ).ToPtr()
  );

  frc2::Trigger scoreCoralButton([this](){ return m_ted.GetR2Button(); });
  scoreCoralButton.OnTrue(SetCoralManipulatorRotation(m_coralManipulator, CoralManipulatorTarget::ScoreCoralL123).ToPtr());
  scoreCoralButton.OnFalse(frc2::ParallelCommandGroup(
      SetCoralElevatorExtension(m_coralElevator, CoralElevatorState::DefaultRetract),
      SetCoralManipulatorRotation(m_coralManipulator, CoralManipulatorTarget::DefaultPosition)
    ).ToPtr()
  );

  frc2::Trigger retractCoralElevatorButton([this](){ return m_ted.GetTouchpadButton(); });
  retractCoralElevatorButton.OnTrue(
    SetCoralElevatorExtension(m_coralElevator, CoralElevatorState::DefaultRetract).ToPtr()
  );

  frc2::Trigger extendCoralElevatorL1Button([this](){ return m_ted.GetCrossButton(); });
  extendCoralElevatorL1Button.OnTrue(
    SetCoralElevatorExtension(m_coralElevator, CoralElevatorState::L1).ToPtr()
  );

  frc2::Trigger extendCoralElevatorL2Button([this](){ return m_ted.GetCircleButton(); });
  extendCoralElevatorL2Button.OnTrue(
    SetCoralElevatorExtension(m_coralElevator, CoralElevatorState::L2).ToPtr()
  );

  frc2::Trigger extendCoralElevatorL3Button([this](){ return m_ted.GetTriangleButton(); });
  extendCoralElevatorL3Button.OnTrue(
    SetCoralElevatorExtension(m_coralElevator, CoralElevatorState::L3).ToPtr()
  );

  frc2::Trigger extendCoralElevatorL4Button([this](){ return m_ted.GetSquareButton(); });
  extendCoralElevatorL4Button.OnTrue(
    SetCoralElevatorExtension(m_coralElevator, CoralElevatorState::L4).ToPtr()
  );
}

void RobotContainer::RegisterAutoCommands() {
  // Start of Auto Events
  pathplanner::NamedCommands::registerCommand("PrintAutoMessage", frc2::InstantCommand([this]{
    for (int i = 0; i < 10; i++) {
      std::cout << "Auto started/ended\n"; }},{}).ToPtr());

  // pathplanner::NamedCommands::registerCommand("AlgaeIntakeRunExtend", frc2::SequentialCommandGroup(
  //     frc2::InstantCommand([this]{
  //       m_algaeIntake->SetRollerPower(AlgaeIntakeConstants::kRollerIntakePower);
  //     }, {m_algaeIntake}),
  //     frc2::ParallelRaceGroup(
  //       frc2::WaitCommand(AutoConstants::kAlgaeIntakeMaxExtendTime),
  //       SetAlgaeIntakeRotation(m_algaeIntake, AlgaeIntakeState::IntakeAlgae)
  //     )
  //   ).ToPtr()
  // );

  // pathplanner::NamedCommands::registerCommand("AlgaeIntakeStopRetract", frc2::SequentialCommandGroup(
  //     frc2::InstantCommand([this]{
  //       m_algaeIntake->SetRollerPower(0.0);
  //     }, {m_algaeIntake}),
  //     frc2::ParallelRaceGroup(
  //       frc2::WaitCommand(AutoConstants::kAlgaeIntakeMaxRetractTime),
  //       SetAlgaeIntakeRotation(m_algaeIntake, AlgaeIntakeState::DefaultRetract)
  //     )
  //   ).ToPtr()
  // );

   pathplanner::NamedCommands::registerCommand("ScoreCoral", frc2::SequentialCommandGroup(
      SetCoralManipulatorRotation(m_coralManipulator, CoralManipulatorTarget::ScoreCoralL123),
      frc2::WaitCommand(AutoConstants::kCoralIntakeScoreTime),
      frc2::ParallelCommandGroup(
        SetCoralElevatorExtension(m_coralElevator, CoralElevatorState::DefaultRetract),
        SetCoralManipulatorRotation(m_coralManipulator, CoralManipulatorTarget::DefaultPosition)
      )
    ).ToPtr()
  );
  
  pathplanner::NamedCommands::registerCommand("RaiseCoralL1", frc2::ParallelRaceGroup(
      SetCoralElevatorExtension(m_coralElevator, CoralElevatorState::L1),
      frc2::WaitCommand(AutoConstants::kCoralElevatorMaxIntakeExtendTime)
    ).ToPtr()
  );

  pathplanner::NamedCommands::registerCommand("RaiseCoralL2", frc2::ParallelRaceGroup(
      SetCoralElevatorExtension(m_coralElevator, CoralElevatorState::L2),
      frc2::WaitCommand(AutoConstants::kCoralElevatorMaxIntakeExtendTime)
    ).ToPtr()
  );

  pathplanner::NamedCommands::registerCommand("RaiseCoralL3", frc2::ParallelRaceGroup(
      SetCoralElevatorExtension(m_coralElevator, CoralElevatorState::L3),
      frc2::WaitCommand(AutoConstants::kCoralElevatorMaxIntakeExtendTime)
    ).ToPtr()
  );

  pathplanner::NamedCommands::registerCommand("RaiseCoralL4", frc2::ParallelRaceGroup(
      SetCoralElevatorExtension(m_coralElevator, CoralElevatorState::L4),
      frc2::WaitCommand(AutoConstants::kCoralElevatorMaxIntakeExtendTime)
    ).ToPtr()
  );

  pathplanner::NamedCommands::registerCommand("CoralMechanismIntakeCoralStation", frc2::SequentialCommandGroup(
      frc2::ParallelCommandGroup(
        frc2::ParallelRaceGroup(
          frc2::WaitCommand(AutoConstants::kCoralElevatorMaxIntakeExtendTime),
          SetCoralElevatorExtension(m_coralElevator, CoralElevatorState::Intake)
        ),
        frc2::ParallelRaceGroup(
          frc2::WaitCommand(AutoConstants::kCoralManipulatorMaxIntakeExtendTime),
          SetCoralManipulatorRotation(m_coralManipulator, CoralManipulatorTarget::Intake)
        )
      )
    ).ToPtr()
  );

  pathplanner::NamedCommands::registerCommand("CoralMechanismDefault", frc2::ParallelCommandGroup(
      frc2::ParallelRaceGroup(
        frc2::WaitCommand(AutoConstants::kCoralElevatorMaxRetractTime),
        SetCoralElevatorExtension(m_coralElevator, CoralElevatorState::DefaultRetract)
      ),
      frc2::ParallelRaceGroup(
        frc2::WaitCommand(AutoConstants::kCoralManipulatorMaxRetractTime),
        SetCoralManipulatorRotation(m_coralManipulator, CoralManipulatorTarget::DefaultPosition)
      )
    ).ToPtr()
  );
} 

void RobotContainer::CreateAutoPaths() {
  // TODO if pathplanner automatic auto populating doesnt work, add back autonomouspaths array
  // for (auto autoPath : AutoConstants::kAutonomousPaths) {
  //   m_chooser.AddOption(autoPath, new pathplanner::PathPlannerAuto(std::string{autoPath}));
  //   for (int i = 0; i < 10; i++) {
  //     std::cout << "Constructing path:" << std::string{autoPath} << std::endl;
  //   }
  //   std::cout << "\n";
  // }
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
  auto command = m_chooser.GetSelected();
  command->AddRequirements(m_swerveDrive);
  command->SetName("Auto command");
  return command;
}

void RobotContainer::SetAllCoast() {
  m_swerveDrive->SetBrakeMode(BrakeMode::Coast);
  m_coralElevator->SetExtensionBrakeMode(BrakeMode::Coast);
}

void RobotContainer::SetAllBrake() {
  m_swerveDrive->SetBrakeMode(BrakeMode::Brake);
  m_coralElevator->SetExtensionBrakeMode(BrakeMode::Brake);
}

// motors will coast along

void RobotContainer::SetAllNormalBrakeMode() {
  m_swerveDrive->SetBrakeMode(BrakeMode::Default);
  m_coralElevator->SetExtensionBrakeMode(BrakeMode::Default);
}

void RobotContainer::SetDriveBrake() {
  m_swerveDrive->SetBrakeMode(BrakeMode::Brake);
}

void RobotContainer::SetDriveCoast() {
  m_swerveDrive->SetBrakeMode(BrakeMode::Coast);
}

void RobotContainer::SetElevatorCoast() {
  m_coralElevator->SetExtensionBrakeMode(BrakeMode::Coast);
}

void RobotContainer::PrintActiveCommands() {
  frc::SmartDashboard::PutString("Current command/swerve", m_swerveDrive->GetCurrentCommand()->GetName());
  frc::SmartDashboard::PutString("Current command/coral elevator", m_coralElevator->GetCurrentCommand()->GetName());
  frc::SmartDashboard::PutString("Current command/coral manipulator", m_coralManipulator->GetCurrentCommand()->GetName());
}

void RobotContainer::ConfigureTestBindings() {
  // frc2::Trigger testAutoButton([this](){ return m_test.GetOptionsButton(); });
  // auto command = m_chooser.GetSelected();
  // testAutoButton.OnTrue(command.ToPtr());

  frc2::Trigger resetPoseButton([this](){ return m_test.GetTouchpadButton(); });
  resetPoseButton.OnTrue(frc2::InstantCommand([this]{
    if (frc::DriverStation::GetAlliance()) {
      if (frc::DriverStation::GetAlliance().value() == frc::DriverStation::Alliance::kBlue)
        m_swerveDrive->SetPose(SwerveDriveConstants::kBlueResetPose, false);
      else
        m_swerveDrive->SetPose(SwerveDriveConstants::kRedResetPose, false);
    }
  },{m_swerveDrive}).ToPtr());

  // frc2::Trigger coralStationAlignButton([this](){ return m_test.GetL2Button(); });
  // coralStationAlignButton.OnTrue(frc2::InstantCommand([this]{
  //   // if (m_driveState == DriveState::HeadingControl) {
  //     m_driveState = DriveState::CoralStationAlign;
  //   // } else {
  //   //   m_driveState = DriveState::HeadingControl;
  //   // }
  //     m_swerveDrive->SetDefaultCommand(Drive(&m_test, m_swerveDrive, m_driveState));
  //   },{m_swerveDrive}).ToPtr()
  // );
  // coralStationAlignButton.OnFalse(frc2::InstantCommand([this]{
  //     m_driveState = DriveState::HeadingControl;
  //     m_swerveDrive->SetDefaultCommand(Drive(&m_test, m_swerveDrive, m_driveState));
  //   },{m_swerveDrive}).ToPtr()
  // );

  // frc2::Trigger algaeIntakeButton([this](){ return m_test.GetL1Button(); });
  // algaeIntakeButton.OnTrue(frc2::SequentialCommandGroup(
  //   frc2::InstantCommand([this]{
  //     m_algaeIntake->SetRollerPower(AlgaeIntakeConstants::kRollerIntakePower);
  //   }, {m_algaeIntake}),
  //   frc2::ParallelRaceGroup(
  //     frc2::WaitCommand(1.0_s),
  //     SetAlgaeIntakeRotation(m_algaeIntake, AlgaeIntakeState::IntakeAlgae)
  //   )
  // ).ToPtr()
  // );
  // algaeIntakeButton.OnFalse(frc2::SequentialCommandGroup(
  //   frc2::InstantCommand([this]{
  //     m_algaeIntake->SetRollerPower(0.0);
  //   }, {m_algaeIntake}),
  //   SetAlgaeIntakeRotation(m_algaeIntake, AlgaeIntakeState::DefaultRetract)
  // ).ToPtr()
  // );

  // frc2::Trigger scoreAlgaeProcessorButton([this](){ return m_test.GetR2Button(); });
  // scoreAlgaeProcessorButton.OnTrue(frc2::ParallelCommandGroup(
  //   SetAlgaeIntakeRotation(m_algaeIntake, AlgaeIntakeState::ScoreProcessor),
  //   RunAlgaeIntakeRoller(m_algaeIntake, AlgaeIntakeConstants::kRollerScorePower)
  // ).ToPtr()
  // );
  // scoreAlgaeProcessorButton.OnFalse(frc2::ParallelCommandGroup(
  //   SetAlgaeIntakeRotation(m_algaeIntake, AlgaeIntakeState::DefaultRetract),
  //    frc2::InstantCommand([this]{
  //     m_algaeIntake->SetRollerPower(0.0);
  //   }, {m_algaeIntake})
  // ).ToPtr()
  // );

  // frc2::Trigger testCoralManipulatorPowerButton([this](){ return m_test.GetL2Button(); });
  // testCoralManipulatorPowerButton.OnTrue(frc2::InstantCommand([this]{
  //   m_coralManipulator->SetRotationPower(0.1);
  //   },{m_coralManipulator}).ToPtr()
  // );
  // testCoralManipulatorPowerButton.OnFalse(frc2::InstantCommand([this]{
  //   m_coralManipulator->SetRotationPower(0.0);
  //   },{m_coralManipulator}).ToPtr()
  // );

  frc2::Trigger intakeCoralButton([this](){ return m_test.GetL2Button(); });
  intakeCoralButton.OnTrue(frc2::ParallelCommandGroup(
      SetCoralElevatorExtension(m_coralElevator, CoralElevatorState::Intake),
      SetCoralManipulatorRotation(m_coralManipulator, CoralManipulatorTarget::Intake)
    ).ToPtr()
  );
  intakeCoralButton.OnFalse(frc2::ParallelCommandGroup(
      SetCoralElevatorExtension(m_coralElevator, CoralElevatorState::DefaultRetract),
      SetCoralManipulatorRotation(m_coralManipulator, CoralManipulatorTarget::DefaultPosition)
    ).ToPtr()
  );

  frc2::Trigger scoreCoralButton([this](){ return m_test.GetR2Button(); });
  scoreCoralButton.OnTrue(SetCoralManipulatorRotation(m_coralManipulator, CoralManipulatorTarget::ScoreCoralL123).ToPtr());
  scoreCoralButton.OnFalse(frc2::ParallelCommandGroup(
      SetCoralElevatorExtension(m_coralElevator, CoralElevatorState::DefaultRetract),
      SetCoralManipulatorRotation(m_coralManipulator, CoralManipulatorTarget::DefaultPosition)
    ).ToPtr()
  );

  // // 5 buttons below just set height of elevator to desired height when pressed
  frc2::Trigger retractCoralElevatorButton([this](){ return m_test.GetTouchpadButton(); });
  retractCoralElevatorButton.OnTrue(
    SetCoralElevatorExtension(m_coralElevator, CoralElevatorState::DefaultRetract).ToPtr()
  );

  frc2::Trigger extendCoralElevatorL1Button([this](){ return m_test.GetCrossButton(); });
  extendCoralElevatorL1Button.OnTrue(
    SetCoralElevatorExtension(m_coralElevator, CoralElevatorState::L1).ToPtr()
  );

  frc2::Trigger extendCoralElevatorL2Button([this](){ return m_test.GetCircleButton(); });
  extendCoralElevatorL2Button.OnTrue(
    SetCoralElevatorExtension(m_coralElevator, CoralElevatorState::L2).ToPtr()
  );

  frc2::Trigger extendCoralElevatorL3Button([this](){ return m_test.GetTriangleButton(); });
  extendCoralElevatorL3Button.OnTrue(
    SetCoralElevatorExtension(m_coralElevator, CoralElevatorState::L3).ToPtr()
  );

  frc2::Trigger extendCoralElevatorL4Button([this](){ return m_test.GetSquareButton(); });
  extendCoralElevatorL4Button.OnTrue(
    SetCoralElevatorExtension(m_coralElevator, CoralElevatorState::L4).ToPtr()
  );
}

// no matter how nice ethan might seem, when you least expect it he will slap you with a piece of chicken and eat you in a bucket