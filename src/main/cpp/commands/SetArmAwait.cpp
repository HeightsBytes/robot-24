// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SetArmAwait.h"

SetArmAwait::SetArmAwait(ArmSubsystem* arm, ArmSubsystem::State target)
    : m_arm(arm), m_target(target) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(arm);
}

// Called when the command is initially scheduled.
void SetArmAwait::Initialize() {
  m_arm->SetTargetState(m_target);
}

// Called repeatedly when this Command is scheduled to run
void SetArmAwait::Execute() {}

// Called once the command ends or is interrupted.
void SetArmAwait::End(bool interrupted) {}

// Returns true when the command should end.
bool SetArmAwait::IsFinished() {
  return m_arm->AtTarget();
}
