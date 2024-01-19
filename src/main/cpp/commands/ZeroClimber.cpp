// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ZeroClimber.h"

#include "Constants.h"

ZeroClimber::ZeroClimber(ClimbSubsystem* climber)
    : m_climber(climber), m_leftZeroed(false), m_rightZeroed(false) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(climber);
}

// Called when the command is initially scheduled.
void ZeroClimber::Initialize() {
  m_climber->m_motorLeft.Set(-.05);
  m_climber->m_motorRight.Set(-.05);
}

// Called repeatedly when this Command is scheduled to run
void ZeroClimber::Execute() {
  if (m_climber->m_motorLeft.GetOutputCurrent() >
      ClimbConstants::kZeroingCurrentThreshold.value()) {
    m_climber->m_motorLeft.Set(0);
    m_leftZeroed = true;
  }
  if (m_climber->m_motorRight.GetOutputCurrent() >
      ClimbConstants::kZeroingCurrentThreshold.value()) {
    m_climber->m_motorLeft.Set(0);
    m_rightZeroed = true;
  }
}

// Called once the command ends or is interrupted.
void ZeroClimber::End(bool interrupted) {
  if (m_leftZeroed && m_rightZeroed) {
    m_climber->m_zeroed = true;
  }
}

// Returns true when the command should end.
bool ZeroClimber::IsFinished() {
  return m_leftZeroed && m_rightZeroed;
}
