// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <string>
#include <units/length.h>
#include <wpi/sendable/SendableBuilder.h>
#include <frc2/command/CommandPtr.h>

class TrapSubsystem : public frc2::SubsystemBase {
 public:

  enum class State {
    kStow,
    kDeployed,
    kSwitching
  };

  TrapSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  State GetActualState() const { return m_actual; }
  State GetTargetState() const { return m_target; }

  double GetPosition() const { return m_encoder.GetPosition(); }

  void SetTargetState(State state) { m_target = state; }

  frc2::CommandPtr SetTargetStateCMD(State state) {
    return this->RunOnce(
      [this, state] { SetTargetState(state); }
    );
  } 

  void InitSendable(wpi::SendableBuilder& builder) override; 

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  std::string ToStr(State state) const;

  double ToOutput(State state) const;

  void ControlLoop();
  void CheckState();

  rev::CANSparkMax m_motor;
  rev::SparkRelativeEncoder m_encoder;
  rev::SparkPIDController m_controller;

  State m_actual;
  State m_target;
};
