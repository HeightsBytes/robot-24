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

  m_leftMotor.BurnFlash();
  m_rightMotor.BurnFlash();
}

// This method will be called once per scheduler run
void ClimbSubsystem::Periodic() {}
