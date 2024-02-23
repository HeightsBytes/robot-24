// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/controller/PIDController.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include <functional>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/VisionSubsystem.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class DriveAndTrack : public frc2::CommandHelper<frc2::Command, DriveAndTrack> {
 public:
  DriveAndTrack(DriveSubsystem* drive, std::function<double()> leftY,
                std::function<double()> leftX);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  DriveSubsystem* m_drive;
  VisionSubsystem& m_vision;
  std::function<double()> m_leftY;
  std::function<double()> m_leftX;
  static constexpr double kP = 0, kI = 0, kD = 0;
  frc::PIDController m_controller{kP, kI, kD};
};
