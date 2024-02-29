// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/Commands.h>
#include <frc2/command/button/RobotModeTriggers.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>

#include <memory>
#include <utility>

#include "commands/DefaultDrive.h"
#include "commands/DriveAndTrack.h"
#include "commands/SetArmAwait.h"
#include "commands/SetRPMAwait.h"

using pathplanner::NamedCommands;

RobotContainer::RobotContainer() {
  frc::SmartDashboard::PutData("Shooter", &m_shooter);
  frc::SmartDashboard::PutData("Arm", &m_arm);
  frc::SmartDashboard::PutData("Auto Chooser", &m_chooser);
  frc::SmartDashboard::PutData("Drive", &m_drive);
  frc::SmartDashboard::PutData("Intake", &m_intake);

  NamedCommands::registerCommand(
      "rev_shooter",
      SetRPMAwait(&m_shooter, ShooterSubsystem::State::kSpeaker).ToPtr());
  NamedCommands::registerCommand(
      "idle_shooter",
      m_shooter.SetTargetStateCMD(ShooterSubsystem::State::kIdle));
  NamedCommands::registerCommand(
      "aim_arm", SetArmAwait(&m_arm, ArmSubsystem::State::kTargetting).ToPtr());
  NamedCommands::registerCommand(
      "stow_arm", SetArmAwait(&m_arm, ArmSubsystem::State::kStow).ToPtr());

  // Configure the button bindings
  ConfigureDriverButtons();
  ConfigureOperatorButtons();

  m_drive.SetDefaultCommand(DefaultDrive(
      &m_drive, [this] { return m_driverController.GetLeftY(); },
      [this] { return m_driverController.GetLeftX(); },
      [this] { return m_driverController.GetRightX(); }));
}

void RobotContainer::ConfigureDriverButtons() {
  m_dleftTrigger
      .WhileTrue(DriveAndTrack(
                     &m_drive, [this] { return m_driverController.GetLeftY(); },
                     [this] { return m_driverController.GetLeftX(); })
                     .ToPtr())
      .OnTrue(m_arm.SetTargetStateCMD(ArmSubsystem::State::kTargetting)
                  .AlongWith(m_shooter.SetTargetStateCMD(
                      ShooterSubsystem::State::kSpeaker)))
      .OnFalse(
          m_shooter.SetTargetStateCMD(ShooterSubsystem::State::kStopped)
              .AlongWith(m_arm.SetTargetStateCMD(ArmSubsystem::State::kStow)));

  m_drightTrigger.OnTrue(m_shooter.SetFeederCMD(1.0))
      .OnFalse(
          m_shooter.SetTargetStateCMD(ShooterSubsystem::State::kStopped)
              .AlongWith(m_arm.SetTargetStateCMD(ArmSubsystem::State::kStow)
                             .AndThen(m_shooter.SetFeederCMD(0))));

  
}

void RobotContainer::ConfigureOperatorButtons() {}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::None();
}
