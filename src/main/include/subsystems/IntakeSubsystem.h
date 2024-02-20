// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkFlex.h>
#include <rev/CANSparkMax.h>

class IntakeSubsystem : public frc2::SubsystemBase {
 public:
  IntakeSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  [[nodiscard]]
  frc2::CommandPtr SetPivot(double set) {
    return this->RunOnce([this, set] { m_pivot.Set(set); });
  }

  [[nodiscard]]
  frc2::CommandPtr SetIntake(double set) {
    return this->RunOnce([this, set] { m_intake.Set(set); });
  }

 private:
  rev::CANSparkFlex m_intake;
  rev::CANSparkMax m_pivot;
};
