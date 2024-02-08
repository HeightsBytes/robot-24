// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ShootNote.h"

#include <fmt/core.h>

ShootNote::ShootNote(ShooterSubsystem* shooter) : m_shooter(shooter) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(shooter);
}

// Called when the command is initially scheduled.
void ShootNote::Initialize() {
  m_timer.Start();
  m_shooter->SetFeeder(1.0);
  fmt::println("[COMMAND CALL]: Shoot Note Called with RPM 0 {} and RPM 1 {}",
               m_shooter->GetSpeed0().value(), m_shooter->GetSpeed1().value());
}

// Called repeatedly when this Command is scheduled to run
void ShootNote::Execute() {}

// Called once the command ends or is interrupted.
void ShootNote::End(bool interrupted) {
  m_shooter->SetFeeder(0);
  m_shooter->SetTargetState(ShooterSubsystem::State::kIdle);
  m_timer.Stop();
  m_timer.Reset();
}

// Returns true when the command should end.
bool ShootNote::IsFinished() {
  return m_timer.HasElapsed(1_s);
}
