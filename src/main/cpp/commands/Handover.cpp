// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Handover.h"

Handover::Handover(ArmSubsystem* arm, ShooterSubsystem* shooter,
                   IntakeSubsystem* intake)
    : m_arm(arm), m_shooter(shooter), m_intake(intake) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({arm, shooter, intake});
}

// Called when the command is initially scheduled.
void Handover::Initialize() {
  m_arm->SetState(ArmSubsystem::State::kStow);
  m_shooter->SetTargetState(ShooterSubsystem::State::kStopped);
  m_intake->SetPivotState(IntakeSubsystem::PivotState::kStow);
}

// Called repeatedly when this Command is scheduled to run
void Handover::Execute() {
  if (m_arm->GetCurrentState() != ArmSubsystem::State::kStow)
    return;
  if (m_intake->GetCurrentPivotState() != IntakeSubsystem::PivotState::kStow)
    return;

  m_intake->SetIntakeState(IntakeSubsystem::IntakeState::kFeed);
  m_shooter->SetFeeder(0.2);
}

// Called once the command ends or is interrupted.
void Handover::End(bool interrupted) {
  m_intake->SetIntakeState(IntakeSubsystem::IntakeState::kStopped);
  m_shooter->SetFeeder(0.0);
  m_shooter->SetTargetState(ShooterSubsystem::State::kIdle);
}

// Returns true when the command should end.
bool Handover::IsFinished() {
  return m_shooter->HasNote();
}
