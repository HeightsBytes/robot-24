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

RobotContainer::RobotContainer() {
  // frc::SmartDashboard::PutData("Shooter", &m_shooter);
  frc::SmartDashboard::PutData("Arm", &m_arm);
  frc::SmartDashboard::PutData("Auto Chooser", &m_chooser);
  // frc::SmartDashboard::PutData("Drive", &m_drive);

  // Configure the button bindings
  ConfigureDriverButtons();
  ConfigureOperatorButtons();
  ConfigureTriggers();

  m_drive.SetDefaultCommand(DefaultDrive(
      &m_drive, [this] { return m_driverController.GetLeftY(); },
      [this] { return m_driverController.GetLeftX(); },
      [this] { return m_driverController.GetRightX(); },
      [this] { return m_driverController.GetLeftTriggerAxis(); }));
}

void RobotContainer::ConfigureDriverButtons() {
  using enum ShooterSubsystem::State;

  m_driverController.LeftBumper()
      .OnTrue(m_shooter.SetTargetStateCMD(kSpeaker))
      .WhileTrue(DriveAndTrack(
                     &m_drive, [this] { return m_driverController.GetLeftY(); },
                     [this] { return m_driverController.GetLeftX(); })
                     .ToPtr())
      .OnFalse(m_shooter.SetTargetStateCMD(kIdle));

  // m_driverController.Y()
  //     .OnTrue(m_shooter.SetFeederCMD(-0.25).AlongWith(m_intake.SetIntake(0.25)))
  //     .OnFalse(m_shooter.SetFeederCMD(0).AlongWith(m_intake.SetIntake(0)));
  // m_driverController.B().OnTrue(m_arm.SetTargetCMD(0_deg));
  // m_driverController.X().OnTrue(
  // m_arm.SetTargetCMD(ArmConstants::Setpoint::kHandoff));
  // m_driverController.A()
  //     .OnTrue(m_intake.SetIntake(-1))
  //     .OnFalse(m_intake.SetIntake(0));

  // m_dleftTrigger.OnTrue(m_intake.SetPivot(0.3)).OnFalse(m_intake.SetPivot(0));
  // m_driverController.LeftBumper()
  //     .OnTrue(m_intake.SetPivot(-0.3))
  //     .OnFalse(m_intake.SetPivot(0));
}

void RobotContainer::ConfigureOperatorButtons() {}

void RobotContainer::ConfigureTriggers() {
  using enum ShooterSubsystem::State;
  frc2::RobotModeTriggers::Teleop().OnTrue(m_shooter.SetTargetStateCMD(kIdle));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::None();
}
