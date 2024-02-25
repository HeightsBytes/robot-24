// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SetRPMAwait.h"

SetRPMAwait::SetRPMAwait(ShooterSubsystem* shooter, ShooterSubsystem::State target) : m_shooter(shooter), m_target(target) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(shooter);
}

// Called when the command is initially scheduled.
void SetRPMAwait::Initialize() {
  m_shooter->SetTargetState(m_target);
}

// Called repeatedly when this Command is scheduled to run
void SetRPMAwait::Execute() {}

// Called once the command ends or is interrupted.
void SetRPMAwait::End(bool interrupted) {}

// Returns true when the command should end.
bool SetRPMAwait::IsFinished() {
  return m_shooter->ShooterReady();
}
