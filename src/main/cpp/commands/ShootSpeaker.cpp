// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ShootSpeaker.h"

ShootSpeaker::ShootSpeaker(ShooterSubsystem* shooter) : m_shooter(shooter) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({shooter});
}

// Called when the command is initially scheduled.
void ShootSpeaker::Initialize() {
  m_shooter->SetFeeder(1.0);
}

// Called repeatedly when this Command is scheduled to run
void ShootSpeaker::Execute() {}

// Called once the command ends or is interrupted.
void ShootSpeaker::End(bool interrupted) {
  m_shooter->SetFeeder(0.0);
}

// Returns true when the command should end.
bool ShootSpeaker::IsFinished() {
  return m_shooter->HasNote();
}
