// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/RobotStateSubsystem.h"

RobotStateSubsystem::RobotStateSubsystem(ArmSubsystem* arm,
                                         ShooterSubsystem* shooter)
    : m_arm(arm), m_shooter(shooter) {}

// This method will be called once per scheduler run
void RobotStateSubsystem::Periodic() {
  UpdateState();
}

void RobotStateSubsystem::InitSendable(wpi::SendableBuilder& builder) {
  builder.SetSmartDashboardType("Robot State");

  builder.AddStringProperty(
      "Scoring State", [this] { return ToStr(m_state); }, nullptr);
}

RobotStateSubsystem::State RobotStateSubsystem::GetState() const {
  return m_state;
}

bool RobotStateSubsystem::SpeakerPrepped() const {
  return m_state == State::kSpeakerPrepped;
}

bool RobotStateSubsystem::AmpPrepped() const {
  return m_state == State::kAmpPrepped;
}

frc2::Trigger RobotStateSubsystem::SpeakerPreppedTrigger() {
  return frc2::Trigger([this] { return SpeakerPrepped(); });
}

frc2::Trigger RobotStateSubsystem::AmpPreppedTrigger() {
  return frc2::Trigger([this] { return AmpPrepped(); });
}

std::string RobotStateSubsystem::ToStr(State state) const {
  switch (state) {
    default:
    case State::kIdle:
      return "Idle";
      break;

    case State::kAmpPrepped:
      return "Amp Prepped";
      break;

    case State::kSpeakerPrepped:
      return "Speaker Prepped";
      break;

    case State::kTrapPrepped:
      return "Trap Prepped";
      break;
  }
}

void RobotStateSubsystem::UpdateState() {
  // Check speaker
  if (m_arm->GetCurrentState() == ArmSubsystem::State::kTargeting &&
      m_shooter->GetCurrentState() == ShooterSubsystem::State::kSpeaker) {
    m_state = State::kSpeakerPrepped;
    return;
  }

  // Check ampo
  if (m_arm->GetCurrentState() == ArmSubsystem::State::kAmp &&
      m_shooter->GetCurrentState() == ShooterSubsystem::State::kTrapAmp) {
    m_state = State::kAmpPrepped;
    return;
  }

  if (m_arm->GetCurrentState() == ArmSubsystem::State::kStow &&
      m_shooter->GetCurrentState() == ShooterSubsystem::State::kIdle) {
    m_state = State::kIdle;
    return;
  }

  m_state = State::kMoving;
}
