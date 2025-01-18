// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/AlgaeIntake.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class RunAlgaeIntakeRoller
    : public frc2::CommandHelper<frc2::Command, RunAlgaeIntakeRoller> {
 public:
 
  RunAlgaeIntakeRoller(AlgaeIntake *algaeIntake, double rollerPower);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  AlgaeIntake *m_algaeIntake;
  double m_rollerPower;
};
