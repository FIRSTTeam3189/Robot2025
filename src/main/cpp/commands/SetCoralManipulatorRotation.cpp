// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SetCoralManipulatorRotation.h"

SetCoralManipulatorRotation::SetCoralManipulatorRotation(CoralManipulator *coralManipulator, CoralManipulatorTarget target) :
m_coralManipulator(coralManipulator),
m_target(target) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(coralManipulator);
}

// Called when the command is initially scheduled.
void SetCoralManipulatorRotation::Initialize() {
  m_coralManipulator->SetState(CoralManipulatorState::GoTarget, m_target);
}

// Called repeatedly when this Command is scheduled to run
void SetCoralManipulatorRotation::Execute() {}

// Called once the command ends or is interrupted.
void SetCoralManipulatorRotation::End(bool interrupted) {}

// Returns true when the command should end.
bool SetCoralManipulatorRotation::IsFinished() {
  if (abs(m_coralManipulator->GetCurrentTargetAngle().value() - m_coralManipulator->GetRotation().value()) < CoralManipulatorConstants::kRotationAngleTolerance.value()) {
    m_coralManipulator->SetState(CoralManipulatorState::HoldPosition);
    return true;
  }
  return false;
}