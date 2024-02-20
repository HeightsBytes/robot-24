// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IntakeSubsystem.h"

IntakeSubsystem::IntakeSubsystem()
    : m_intake(20, rev::CANSparkFlex::MotorType::kBrushless),
      m_pivot(21, rev::CANSparkMax::MotorType::kBrushless) {}

// This method will be called once per scheduler run
void IntakeSubsystem::Periodic() {}
