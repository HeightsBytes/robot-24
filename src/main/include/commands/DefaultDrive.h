// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/MathUtil.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/button/CommandXboxController.h>

#include <functional>

#include "subsystems/DriveSubsystem.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class DefaultDrive : public frc2::CommandHelper<frc2::Command, DefaultDrive> {
 public:
  DefaultDrive(DriveSubsystem* drive, std::function<double()> leftY,
               std::function<double()> leftX, std::function<double()> rightX,
               std::function<bool()> slowMode = [] {return false;});

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  DriveSubsystem* m_drive;
  std::function<double()> m_leftY;
  std::function<double()> m_leftX;
  std::function<double()> m_rightX;
  std::function<bool()> m_slowMode;
};
