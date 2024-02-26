// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ClimbSubsystem.h"

#include "Constants.h"

ClimbSubsystem::ClimbSubsystem()
    : m_leftMotor(ClimbConstants::kMotorLeftID,
                  rev::CANSparkMax::MotorType::kBrushless),
      m_rightMotor(ClimbConstants::kMotorRightID,
                   rev::CANSparkMax::MotorType::kBrushless) {
  m_leftMotor.RestoreFactoryDefaults();
  m_rightMotor.RestoreFactoryDefaults();

  m_leftMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_rightMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
}

// This method will be called once per scheduler run
void ClimbSubsystem::Periodic() {}

frc2::CommandPtr ClimbSubsystem::SetLeftMotorCMD(double set) {
  return this->RunOnce([this, set] { SetLeftMotor(set); });
}

frc2::CommandPtr ClimbSubsystem::SetRightMotorCMD(double set) {
  return this->RunOnce([this, set] { SetRightMotor(set); });
}
