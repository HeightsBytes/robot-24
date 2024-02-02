// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ScoreAmp.h"

ScoreAmp::ScoreAmp(ShooterSubsystem* shooter) : m_shooter(shooter) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(shooter);
}

// Called when the command is initially scheduled.
void ScoreAmp::Initialize() {
  m_shooter->SetFeeder(1.0);
}

// Called repeatedly when this Command is scheduled to run
void ScoreAmp::Execute() {}

// Called once the command ends or is interrupted.
void ScoreAmp::End(bool interrupted) {
  m_shooter->SetFeeder(0);
}

// Returns true when the command should end.
bool ScoreAmp::IsFinished() {
  return m_shooter->HasNote();
}
