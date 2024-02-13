// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/DigitalInput.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/button/Trigger.h>
#include <rev/CANSparkFlex.h>
#include <rev/CANSparkMax.h>
#include <units/angular_velocity.h>
#include <wpi/sendable/SendableBuilder.h>

#include <string>

#include "Constants.h"

// 2 NEO Vortex
class ShooterSubsystem : public frc2::SubsystemBase {
 public:
  enum class State { kStopped, kIdle, kTrapAmp, kSpeaker, kSwitching };

  ShooterSubsystem();

  void Periodic() override;

  inline bool ShooterReady() const {
    return (GetActualState0() == GetTargetState()) && (GetActualState1() == GetTargetState());
  }

  inline units::revolutions_per_minute_t GetSpeed0() const {
    return units::revolutions_per_minute_t(m_encoder0.GetVelocity());
  }
  inline units::revolutions_per_minute_t GetSpeed1() const {
    return units::revolutions_per_minute_t(m_encoder1.GetVelocity());
  }

  inline State GetTargetState() const { return m_target; }
  inline State GetActualState0() const { return m_actual0; }
  inline State GetActualState1() const { return m_actual1; }

  inline void SetTargetState(State target) { m_target = target; }
  void SetFeeder(double setpoint);

  [[nodiscard]]
  frc2::CommandPtr SetTargetStateCMD(State target);

  [[nodiscard]]
  frc2::CommandPtr SetFeederCMD(double set) {
    return this->RunOnce([this, set] { SetFeeder(set); });
  }

  frc2::Trigger ShooterReadyTrigger() {
    return frc2::Trigger([this] { return ShooterReady(); });
  }

  void InitSendable(wpi::SendableBuilder& builder) override;

 private:
  void CheckState0();
  void CheckState1();

  std::string ToStr(State state) const;

  units::revolutions_per_minute_t ToRPM0(State state) const;
  units::revolutions_per_minute_t ToRPM1(State state) const;

  rev::CANSparkFlex m_leftFlywheel;
  rev::CANSparkFlex m_rightFlywheel;

  rev::SparkRelativeEncoder m_encoder0;
  rev::SparkRelativeEncoder m_encoder1;

  rev::SparkPIDController m_controller0;
  rev::SparkPIDController m_controller1;

  rev::CANSparkMax m_leftFeeder;
  rev::CANSparkMax m_rightFeeder;

  State m_actual0;
  State m_actual1;
  State m_target;

  /***TUNING***/

  // bool m_tuning = false;
  // double kP = ShooterConstants::kP;
  // double kI = ShooterConstants::kI;
  // double kD = ShooterConstants::kD;
  // double kFF = ShooterConstants::kFF;
  // double RPMSetpoint = ShooterConstants::Setpoint::kShooting.value();
};
