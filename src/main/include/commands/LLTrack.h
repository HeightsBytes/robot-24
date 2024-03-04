// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include <functional>

#include "subsystems/DriveSubsystem.h"
#include <frc/controller/PIDController.h>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class LLTrack
    : public frc2::CommandHelper<frc2::Command, LLTrack> {
 public:
  LLTrack(DriveSubsystem* drive, std::function<double()> leftY, std::function<double()> leftX);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:
    DriveSubsystem* m_drive;
    std::function<double()> m_leftY;
    std::function<double()> m_leftX;

    frc::PIDController m_controller{0.05, 0, 0};

    bool m_hasTarget;
    units::degree_t m_angle;
};
