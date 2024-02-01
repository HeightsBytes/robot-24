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
#include "subsystems/RobotStateSubsystem.h"
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/VisionSubsystem.h"

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

  frc::PowerDistribution m_pdp{0, frc::PowerDistribution::ModuleType::kCTRE};

  // The robot's subsystems
  VisionSubsystem& m_vision = VisionSubsystem::GetInstance();
  DriveSubsystem m_drive;
  IntakeSubsystem m_intake;
  ClimbSubsystem m_climber;
  ArmSubsystem m_arm{[this]() -> frc::Pose2d { return m_drive.GetPose(); }};
  ShooterSubsystem m_shooter;
  RobotStateSubsystem m_state{&m_arm,
                              &m_shooter};  // does not command, simply uses
                                            // dependency injection for state

  frc::SendableChooser<std::string> m_chooser;

  // Triggers

  frc2::Trigger m_dleftTrigger{
      [this] { return m_driverController.GetLeftTriggerAxis() > 0.6; }};
  frc2::Trigger m_drightTrigger{
      [this] { return m_driverController.GetRightTriggerAxis() > 0.6; }};

  // True if not zeroed
  frc2::Trigger m_zeroClimberTrigger{[this] { return !m_climber.IsZeroed(); }};

  void ConfigureDriverButtons();

  void ConfigureOperatorButtons();

  void ConfigureTriggers();
};
