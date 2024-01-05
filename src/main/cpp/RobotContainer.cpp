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

#include "commands/DefaultDrive.h"
#include "subsystems/DriveSubsystem.h"

RobotContainer::RobotContainer() {
  m_chooser.AddOption("Test Auto", "test_auto");
  m_chooser.AddOption("Straight Line", "just_move");
  m_chooser.AddOption("Balance Path", "red_auto");
  m_chooser.AddOption("Crazy Auto", "red_crazy_auto");

  // Other Commands
  pathplanner::NamedCommands::registerCommand(
      "drive_switch", std::move(m_drive.SetGyro(180_deg)));

  frc::SmartDashboard::PutData("PDP", &m_pdp);
  frc::SmartDashboard::PutData("Auto Chooser", &m_chooser);
  frc::SmartDashboard::PutData("Command Scheduler",
                               &frc2::CommandScheduler::GetInstance());

  // Configure the button bindings
  ConfigureDriverButtons();
  ConfigureOperatorButtons();

  // Uses right trigger + left stick axis + right stick axis
  m_drive.SetDefaultCommand(DefaultDrive(
      &m_drive, [this] { return m_driverController.GetLeftY(); },
      [this] { return m_driverController.GetLeftX(); },
      [this] { return m_driverController.GetRightX(); },
      [this] { return m_driverController.GetRightTriggerAxis(); }));
}

void RobotContainer::ConfigureDriverButtons() {
  m_driverController.A().OnTrue(frc2::cmd::Print("Example!"));
}

void RobotContainer::ConfigureOperatorButtons() {
  m_operatorController.A().OnTrue(frc2::cmd::Print("Example!"));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return pathplanner::PathPlannerAuto(m_chooser.GetSelected()).ToPtr();
}
