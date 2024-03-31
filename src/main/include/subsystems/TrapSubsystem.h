// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <string>
#include <units/length.h>
#include <wpi/sendable/SendableBuilder.h>

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

  units::meter_t GetPosition() const { return units::meter_t(m_encoder.GetPosition()); }

  void InitSendable(wpi::SendableBuilder& builder) override; 

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  std::string ToStr(State state) const;

  units::meter_t ToOutput(State state) const;

  void ControlLoop();
  void CheckState();

  rev::CANSparkMax m_motor;
  rev::SparkRelativeEncoder m_encoder;
  rev::SparkPIDController m_controller;

  State m_actual;
  State m_target;
};
