// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/controller/PIDController.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include <functional>

#include "subsystems/ArmSubsystem.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/VisionSubsystem.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class RequestSpeaker
    : public frc2::CommandHelper<frc2::Command, RequestSpeaker> {
 public:
  RequestSpeaker(ArmSubsystem* arm, DriveSubsystem* drive,
                 ShooterSubsystem* shooter, std::function<double()> leftX,
                 std::function<double()> leftY);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  ArmSubsystem* m_arm;
  DriveSubsystem* m_drive;
  ShooterSubsystem* m_shooter;
  VisionSubsystem& m_vision;

  frc::PIDController m_controller;

  std::function<double()> m_leftX, m_leftY;
};
