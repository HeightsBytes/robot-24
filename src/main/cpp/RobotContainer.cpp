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
#include <frc2/command/button/POVButton.h>

#include <memory>
#include <utility>

#include "commands/DefaultDrive.h"
#include "commands/DriveAndTrack.h"
#include "commands/SetArmAwait.h"
#include "commands/SetRPMAwait.h"
#include "commands/SetIntakeAwait.h"
#include "commands/LLTrack.h"
#include "commands/Commands.h"

using pathplanner::NamedCommands;

RobotContainer::RobotContainer() {
  frc::SmartDashboard::PutData("Shooter", &m_shooter);
  frc::SmartDashboard::PutData("Arm", &m_arm);
//   frc::SmartDashboard::PutData("Auto Chooser", &m_chooser);
  frc::SmartDashboard::PutData("Drive", &m_drive);
  frc::SmartDashboard::PutData("Intake", &m_intake);

  NamedCommands::registerCommand(
      "rev_shooter",
      Commands::RevShooter(&m_shooter));

  NamedCommands::registerCommand(
      "aim_arm",
      m_arm.SetTargetStateCMD(ArmSubsystem::State::kTargetting));

  NamedCommands::registerCommand("flip_gyro", m_drive.SetGyro(180_deg));
  
  NamedCommands::registerCommand("shoot_note", Commands::ShootNote(&m_shooter));

  NamedCommands::registerCommand("handoff", Commands::Handoff(&m_arm, &m_intake, &m_shooter));

  NamedCommands::registerCommand("deploy_intake", m_intake.DeployIntakeCMD());
  NamedCommands::registerCommand("stow_intake", m_intake.StowIntakeCMD());

  // Configure the button bindings
  ConfigureDriverButtons();
  ConfigureOperatorButtons();

  m_drive.SetDefaultCommand(DefaultDrive(
      &m_drive, [this] { return m_driverController.GetLeftY(); },
      [this] { return m_driverController.GetLeftX(); },
      [this] { return m_driverController.GetRightX(); }));
}

void RobotContainer::ConfigureDriverButtons() {

  
  m_driverController.A().OnTrue(m_arm.SetTargetStateCMD(ArmSubsystem::State::kStow));

  m_driverController.RightBumper().OnTrue(m_intake.DeployIntakeCMD()).OnFalse(m_intake.StowIntakeCMD());

  m_driverController.LeftBumper().OnTrue(Commands::Handoff(&m_arm, &m_intake, &m_shooter));

  m_dleftTrigger.OnTrue(m_arm.SetTargetStateCMD(ArmSubsystem::State::kTargetting)
    .AlongWith(LLTrack(&m_drive, [this] {return m_driverController.GetLeftY();}, [this] {return m_driverController.GetLeftX();}).ToPtr()))
    .OnFalse(m_arm.SetTargetStateCMD(ArmSubsystem::State::kStow).AlongWith(
      DefaultDrive(
      &m_drive, [this] { return m_driverController.GetLeftY(); },
      [this] { return m_driverController.GetLeftX(); },
      [this] { return m_driverController.GetRightX(); }).ToPtr()
    ));
  m_drightTrigger.OnTrue(Commands::RevShooter(&m_shooter).AndThen(Commands::ShootNote(&m_shooter)));

  // m_drightTrigger.OnTrue(m_shooter.SetTargetStateCMD(ShooterSubsystem::State::kSpeaker)).OnFalse(m_shooter.SetTargetStateCMD(ShooterSubsystem::State::kStopped));

}

void RobotContainer::ConfigureOperatorButtons() {
  frc2::POVButton(&m_operatorController, 0).OnTrue(m_climber.SetLeftMotorCMD(-0.5)).OnFalse(m_climber.SetLeftMotorCMD(0));
  frc2::POVButton(&m_operatorController, 180).OnTrue(m_climber.SetLeftMotorCMD(0.5)).OnFalse(m_climber.SetLeftMotorCMD(0));

  m_operatorController.Y().OnTrue(m_climber.SetRightMotorCMD(0.5)).OnFalse(m_climber.SetRightMotorCMD(0));
  m_operatorController.A().OnTrue(m_climber.SetRightMotorCMD(-0.5)).OnFalse(m_climber.SetRightMotorCMD(0));

}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return pathplanner::PathPlannerAuto("2N-2").ToPtr();
}
