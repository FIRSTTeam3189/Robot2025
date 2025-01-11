// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SetCoralElevatorExtension.h"

SetCoralElevatorExtension::SetCoralElevatorExtension(CoralElevator *CoralElevator, CoralElevatorState state) :
m_CoralElevator(CoralElevator),
m_state(state) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(CoralElevator);
}

// Called when the command is initially scheduled.
void SetCoralElevatorExtension::Initialize() {
  m_CoralElevator->SetState(m_state);
}

// Called repeatedly when this Command is scheduled to run
void SetCoralElevatorExtension::Execute() {}

// Called once the command ends or is interrupted.
void SetCoralElevatorExtension::End(bool interrupted) {}

// Returns true when the command should end.
bool SetCoralElevatorExtension::IsFinished() {
  if (abs(m_CoralElevator->GetCurrentTargetHeight().value() - m_CoralElevator->GetExtension().value()) < CoralElevatorConstants::kExtensionHeightTolerance.value()) {
    return true;
  }
  return false;
}
