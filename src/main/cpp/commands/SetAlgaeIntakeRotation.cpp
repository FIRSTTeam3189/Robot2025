// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SetAlgaeIntakeRotation.h"

SetAlgaeIntakeRotation::SetAlgaeIntakeRotation(AlgaeIntake *algaeIntake, AlgaeIntakeState state, AlgaeIntakeTarget target) :
m_algaeIntake(algaeIntake),
m_state(state),
m_target(target) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(algaeIntake);
}

// Called when the command is initially scheduled.
void SetAlgaeIntakeRotation::Initialize() {
  m_algaeIntake->SetState(m_state, m_target);
}

// Called repeatedly when this Command is scheduled to run
void SetAlgaeIntakeRotation::Execute() {}

// Called once the command ends or is interrupted.
void SetAlgaeIntakeRotation::End(bool interrupted) {}

// Returns true when the command should end.
bool SetAlgaeIntakeRotation::IsFinished() {
  if (abs(m_algaeIntake->GetCurrentTargetAngle().value() - m_algaeIntake->GetRotation().value()) < AlgaeIntakeConstants::kRotationAngleTolerance.value()) {
    return true;
  }
  return false;
}
