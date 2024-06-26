// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/Commands.h>
#include <frc2/command/button/POVButton.h>
#include <frc2/command/button/RobotModeTriggers.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>

#include <memory>
#include <utility>

#include "commands/Commands.h"
#include "commands/DefaultDrive.h"
#include "commands/LLTrack.h"
#include "commands/SetArmAwait.h"
#include "commands/SetIntakeAwait.h"
#include "commands/SetRPMAwait.h"

using pathplanner::NamedCommands;

RobotContainer::RobotContainer() {
  m_chooser.SetDefaultOption("None", "None");
  m_chooser.AddOption("2N-2", "2N-2");
  m_chooser.AddOption("2N-F5", "2N-F5");
  m_chooser.AddOption("1N-Top", "1N-Top");
  m_chooser.AddOption("2N-Top", "2N-Top");
  frc::SmartDashboard::PutData("Auto Chooser", &m_chooser);

  NamedCommands::registerCommand("rev_shooter",
                                 Commands::RevShooter(&m_shooter));

  NamedCommands::registerCommand(
      "aim_arm", m_arm.SetTargetStateCMD(ArmSubsystem::State::kTargetting));

  NamedCommands::registerCommand("flip_gyro", m_drive.SetGyro(180_deg));

  NamedCommands::registerCommand("shoot_note", Commands::ShootNote(&m_shooter));

  NamedCommands::registerCommand(
      "handoff", Commands::Handoff(&m_arm, &m_intake, &m_shooter));

  NamedCommands::registerCommand("deploy_intake", m_intake.DeployIntakeCMD());
  NamedCommands::registerCommand("stow_intake", m_intake.StowIntakeCMD());
  NamedCommands::registerCommand(
      "stow_arm", m_arm.SetTargetStateCMD(ArmSubsystem::State::kStow));
  NamedCommands::registerCommand(
      "aim_robot", LLTrack(
                       &m_drive, [] { return 0.0; }, [] { return 0.0; })
                       .ToPtr());

  // Configure the button bindings
  ConfigureDriverButtons();
  ConfigureOperatorButtons();

  m_drive.SetDefaultCommand(DefaultDrive(
      &m_drive, [this] { return m_driverController.GetLeftY(); },
      [this] { return m_driverController.GetLeftX(); },
      [this] { return m_driverController.GetRightX(); },
      [this] { return m_driverController.GetLeftStickButton(); }));
  
}

void RobotContainer::ConfigureDriverButtons() {
  m_driverController.A().OnTrue(
      m_arm.SetTargetStateCMD(ArmSubsystem::State::kStow));

  m_driverController.B()
      .OnTrue(
          m_intake.SetIntakeTargetCMD(IntakeSubsystem::IntakeState::kIntaking))
      .OnFalse(
          m_intake.SetIntakeTargetCMD(IntakeSubsystem::IntakeState::kStopped));
  m_driverController.X()
      .OnTrue(
          m_intake.SetIntakeTargetCMD(IntakeSubsystem::IntakeState::kHandoff))
      .OnFalse(
          m_intake.SetIntakeTargetCMD(IntakeSubsystem::IntakeState::kStopped));

  m_driverController.RightBumper()
      .OnTrue(m_intake.DeployIntakeCMD())
      .OnFalse(m_intake.StowIntakeCMD());

  m_driverController.LeftBumper().OnTrue(
      Commands::Handoff(&m_arm, &m_intake, &m_shooter));

  m_dleftTrigger
      .OnTrue(
          m_arm.SetTargetStateCMD(ArmSubsystem::State::kTargetting)
              .AlongWith(LLTrack(
                             &m_drive,
                             [this] { return m_driverController.GetLeftY(); },
                             [this] { return m_driverController.GetLeftX(); })
                             .ToPtr())
              .AlongWith(Commands::RevShooter(&m_shooter)))
      .OnFalse(m_shooter.SetFeederCMD(0).AndThen(
          m_arm.SetTargetStateCMD(ArmSubsystem::State::kStow)
              .AlongWith(DefaultDrive(
                             &m_drive,
                             [this] { return m_driverController.GetLeftY(); },
                             [this] { return m_driverController.GetLeftX(); },
                             [this] { return m_driverController.GetRightX(); })
                             .ToPtr())
              .AlongWith(m_shooter.SetTargetStateCMD(
                  ShooterSubsystem::State::kStopped))));
                  
  (m_shooter.ShooterReadyToFireTrigger() && m_arm.AtSpeakerTargetTrigger() &&
   (m_robotAimedAtSpeaker || m_robotSpeedTrigger) && m_drightTrigger)
      .OnTrue(Commands::ShootNote(&m_shooter));

  frc2::POVButton(&m_driverController, 0)
      .OnTrue(m_arm.SetTargetStateCMD(ArmSubsystem::State::kTrap));

  frc2::POVButton(&m_driverController, 90)
      .OnTrue(Commands::ShootNote(&m_shooter));

  frc2::POVButton(&m_driverController, 180)
      .OnTrue(
          m_shooter.SetTargetStateCMD(ShooterSubsystem::State::kStopped)
              .AlongWith(m_arm.SetTargetStateCMD(ArmSubsystem::State::kStow)));

  frc2::POVButton(&m_driverController, 270)
      .OnTrue(
        m_shooter.SetTargetStateCMD(ShooterSubsystem::State::kTrapAmp)
      );
}

void RobotContainer::ConfigureOperatorButtons() {
  frc2::POVButton(&m_operatorController, 0)
      .OnTrue(m_climber.SetLeftMotorCMD(-1))
      .OnFalse(m_climber.SetLeftMotorCMD(0));
  frc2::POVButton(&m_operatorController, 180)
      .OnTrue(m_climber.SetLeftMotorCMD(1))
      .OnFalse(m_climber.SetLeftMotorCMD(0));

  m_operatorController.Y()
      .OnTrue(m_climber.SetRightMotorCMD(1))
      .OnFalse(m_climber.SetRightMotorCMD(0));
  m_operatorController.A()
      .OnTrue(m_climber.SetRightMotorCMD(-1))
      .OnFalse(m_climber.SetRightMotorCMD(0));

//   m_operatorController.RightBumper().OnTrue(m_trapper.SetTargetStateCMD(TrapSubsystem::State::kDeployed));
//   m_operatorController.LeftBumper().OnTrue(m_trapper.SetTargetStateCMD(TrapSubsystem::State::kStow));

  // m_operatorController.RightBumper().OnTrue(m_trapper.SetTargetCMD(0.5)).OnFalse(m_trapper.StopMotorCMD());
  // m_operatorController.LeftBumper().OnTrue(m_trapper.SetTargetCMD(-0.5)).OnFalse(m_trapper.StopMotorCMD());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  auto selected = m_chooser.GetSelected();
  if (selected == "None") {
    return frc2::cmd::None();
  } else {
    return pathplanner::PathPlannerAuto(selected).ToPtr();
  }
}
