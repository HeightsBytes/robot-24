// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SetIntakeAwait.h"

SetIntakeAwait::SetIntakeAwait(IntakeSubsystem* intake, IntakeSubsystem::PivotState state) : m_intake(intake), m_state(state) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(intake);
}

// Called when the command is initially scheduled.
void SetIntakeAwait::Initialize() {
  m_intake->SetPivotTarget(m_state);
}

// Called repeatedly when this Command is scheduled to run
void SetIntakeAwait::Execute() {}

// Called once the command ends or is interrupted.
void SetIntakeAwait::End(bool interrupted) {}

// Returns true when the command should end.
bool SetIntakeAwait::IsFinished() {
  return m_intake->AtPivotTarget();
}
