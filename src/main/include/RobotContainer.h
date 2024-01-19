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
#include "subsystems/DriveSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
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

  frc::SendableChooser<std::string> m_chooser;

  // Triggers

  void ConfigureDriverButtons();

  void ConfigureOperatorButtons();
};
