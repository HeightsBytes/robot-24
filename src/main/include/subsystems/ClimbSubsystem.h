// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <frc/DigitalInput.h>

class ClimbSubsystem : public frc2::SubsystemBase {
 public:
  ClimbSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void SetLeftMotor(double set) { m_leftRequested = set; }
  void SetRightMotor(double set) { m_rightRequested = set; }

  bool GetLeftSwitch() const { return !m_leftSwitch.Get(); }
  bool GetRightSwitch() const { return !m_rightSwitch.Get(); }

  [[nodiscard]]
  frc2::CommandPtr SetMotorsCMD(double set) {
    return this->RunOnce([this, set] {
      m_leftMotor.Set(set);
      m_rightMotor.Set(-set);
    });
  }

  [[nodiscard]]
  frc2::CommandPtr SetLeftMotorCMD(double set) {
    return this->RunOnce([this, set] { SetLeftMotor(set); });
  }

  [[nodiscard]]
  frc2::CommandPtr SetRightMotorCMD(double set) {
    return this->RunOnce([this, set] { SetRightMotor(set); });
  }

 private:
  rev::CANSparkMax m_leftMotor;
  rev::CANSparkMax m_rightMotor;

  frc::DigitalInput m_leftSwitch;
  frc::DigitalInput m_rightSwitch;

  double m_leftRequested;
  double m_rightRequested;
};
