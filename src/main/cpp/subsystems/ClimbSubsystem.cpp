// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ClimbSubsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants.h"

ClimbSubsystem::ClimbSubsystem()
    : m_leftMotor(ClimbConstants::kMotorLeftID,
                  rev::CANSparkMax::MotorType::kBrushless),
      m_rightMotor(ClimbConstants::kMotorRightID,
                   rev::CANSparkMax::MotorType::kBrushless),
      m_leftSwitch(ClimbConstants::kLeftSwitchID),
      m_rightSwitch(ClimbConstants::kRightSwitchID),
      m_leftRequested(0),
      m_rightRequested(0) {
  m_leftMotor.RestoreFactoryDefaults();
  m_rightMotor.RestoreFactoryDefaults();

  m_leftMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_rightMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

  m_leftMotor.BurnFlash();
  m_rightMotor.BurnFlash();

}

// This method will be called once per scheduler run
void ClimbSubsystem::Periodic() {

  double leftOut;
  if (!GetLeftSwitch()) {
    leftOut = m_leftRequested;
  } else if (m_leftRequested < 0) {
    leftOut = m_leftRequested;
  } else {
    leftOut = 0;
  }

  double rightOut;
  if (!GetRightSwitch()) {
    rightOut = m_rightRequested;
  } else if (m_rightRequested > 0) {
    rightOut = m_rightRequested;
  } else {
    rightOut = 0;
  }

  m_leftMotor.Set(leftOut);
  m_rightMotor.Set(rightOut);

}
