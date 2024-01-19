// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ShooterSubsystem.h"

#include "Constants.h"

// ShooterSubsystem::ShooterSubsystem() :
//     m_motor0(ShooterConstants::kMotor0ID,
//     rev::CANSparkFlex::MotorType::kBrushless),
//     m_motor1(ShooterConstants::kMotor1ID,
//     rev::CANSparkFlex::MotorType::kBrushless),
//     m_encoder0(m_motor0.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor)),

ShooterSubsystem::ShooterSubsystem() = default;

// This method will be called once per scheduler run
void ShooterSubsystem::Periodic() {}
