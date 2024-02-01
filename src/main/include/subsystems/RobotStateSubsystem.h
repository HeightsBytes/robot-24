// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/button/Trigger.h>
#include <wpi/sendable/SendableBuilder.h>

#include "subsystems/ArmSubsystem.h"
#include "subsystems/ShooterSubsystem.h"

#include <string>

class RobotStateSubsystem : public frc2::SubsystemBase {
 public:
  RobotStateSubsystem(ArmSubsystem* arm, ShooterSubsystem* shooter);

  enum class State { kIdle, kSpeakerPrepped, kAmpPrepped, kTrapPrepped, kMoving };

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  State GetState() const;

  bool SpeakerPrepped() const;
  bool AmpPrepped() const;

  frc2::Trigger SpeakerPreppedTrigger();
  frc2::Trigger AmpPreppedTrigger();

  void InitSendable(wpi::SendableBuilder& builder) override;

 private:

  std::string ToStr(State state) const;

  void UpdateState();

  ArmSubsystem* m_arm;
  ShooterSubsystem* m_shooter;

  State m_state;

};
