// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/DutyCycleEncoder.h>
#include <frc/PowerDistribution.h>
#include <frc/XboxController.h>
#include <frc/controller/PIDController.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>

#include <optional>
#include <string>

#include "Constants.h"
#include "subsystems/ArmSubsystem.h"
#include "subsystems/ClimbSubsystem.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/TrapSubsystem.h"
#include "utils/cams/Limelight.h"
#include "utils/Util.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();

 private:
  frc2::CommandXboxController m_driverController{
      OIConstants::kDriverControllerPort};
  frc2::CommandXboxController m_operatorController{
      OIConstants::kOperatorControllerPort};

  ShooterSubsystem m_shooter;
  ArmSubsystem m_arm;
  IntakeSubsystem m_intake;
  DriveSubsystem m_drive;
  ClimbSubsystem m_climber;
  TrapSubsystem m_trapper;

  frc::SendableChooser<std::string> m_chooser;

  // Triggers

  frc2::Trigger m_dleftTrigger{
      [this] { return m_driverController.GetLeftTriggerAxis() > 0.3; }};
  frc2::Trigger m_drightTrigger{
      [this] { return m_driverController.GetRightTriggerAxis() > 0.3; }};

  frc2::Trigger m_oleftTrigger{
      [this] { return m_operatorController.GetLeftTriggerAxis() > 0.3; }};
  frc2::Trigger m_orightTrigger{
      [this] { return m_operatorController.GetRightTriggerAxis() > 0.3; }};

  frc2::Trigger m_robotAimedAtSpeaker{
      [this] { 
        if (hb::LimeLight::HasTarget()) {
          return hb::InRange(0, hb::LimeLight::GetX(), 1.0);
        } else return true;
      }};

  frc2::Trigger m_robotSpeedTrigger{
      [this] {
        return m_drive.GetVelocity().vx.value() < 0.5 && m_drive.GetVelocity().vy.value() < 1.5;
      }
  };

  void ConfigureDriverButtons();

  void ConfigureOperatorButtons();
};
