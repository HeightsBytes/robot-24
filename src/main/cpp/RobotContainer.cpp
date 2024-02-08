// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/Commands.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>

#include <memory>
#include <utility>

#include "commands/ShootNote.h"

RobotContainer::RobotContainer() {
  frc::SmartDashboard::PutData("PDP", &m_pdp);
  frc::SmartDashboard::PutData("Auto Chooser", &m_chooser);
  frc::SmartDashboard::PutData("Command Scheduler",
                               &frc2::CommandScheduler::GetInstance());

  frc::SmartDashboard::PutData("Shooter", &m_shooter);

  // Configure the button bindings
  ConfigureDriverButtons();
  ConfigureOperatorButtons();
  ConfigureTriggers();
}

void RobotContainer::ConfigureDriverButtons() {
  m_driverController.A().OnTrue(
      m_shooter.SetTargetStateCMD(ShooterSubsystem::State::kSpeaker));

  (m_drightTrigger && m_shooter.ShooterReadyTrigger())
      .OnTrue(ShootNote(&m_shooter).ToPtr());
}

void RobotContainer::ConfigureOperatorButtons() {}

void RobotContainer::ConfigureTriggers() {}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::None();
}
