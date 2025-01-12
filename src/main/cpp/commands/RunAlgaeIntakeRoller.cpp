// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RunAlgaeIntakeRoller.h"

RunAlgaeIntakeRoller::RunAlgaeIntakeRoller(AlgaeIntake *algaeIntake, double rollerPower) :
m_algaeIntake(algaeIntake), m_rollerPower(rollerPower) {
  AddRequirements(algaeIntake);
}

// Called when the command is initially scheduled.
void RunAlgaeIntakeRoller::Initialize() {
  m_algaeIntake->SetRollerPower(m_rollerPower);
}

// Called repeatedly when this Command is scheduled to run
void RunAlgaeIntakeRoller::Execute() {}

// Called once the command ends or is interrupted.
void RunAlgaeIntakeRoller::End(bool interrupted) {
  m_algaeIntake->SetRollerPower(0.0);
}

// Returns true when the command should end.
bool RunAlgaeIntakeRoller::IsFinished() {
  return false;
}
