// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/DigitalInput.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkFlex.h>
#include <units/angular_velocity.h>
#include <wpi/sendable/SendableBuilder.h>

#include <string>

// 2 NEO Vortex
class ShooterSubsystem : public frc2::SubsystemBase {
 public:
  enum class State { kStopped, kIdle, kTrapAmp, kSpeaker, kSwitching };

  ShooterSubsystem();

  void Periodic() override;

  bool HasGamePiece() const;

  units::revolutions_per_minute_t GetSpeed0() const;
  units::revolutions_per_minute_t GetSpeed1() const;

  State GetTargetState() const;
  State GetCurrentState() const;

  void SetTargetState(State target);

  [[nodiscard]]
  frc2::CommandPtr SetTargetStateCMD(State target);

  void InitSendable(wpi::SendableBuilder& builder) override;

 private:
  void CheckState();

  std::string ToStr(State state) const;

  units::revolutions_per_minute_t ToRPM(State state) const;

  rev::CANSparkFlex m_leftFlywheel;
  rev::CANSparkFlex m_rightFlywheel;

  rev::SparkRelativeEncoder m_encoder0;
  rev::SparkRelativeEncoder m_encoder1;

  rev::SparkPIDController m_controller0;
  rev::SparkPIDController m_controller1;

  frc::DigitalInput m_beamBreak;

  State m_actual;
  State m_target;
};
