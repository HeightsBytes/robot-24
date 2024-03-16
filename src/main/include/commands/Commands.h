// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>

#include "commands/DefaultDrive.h"
#include "commands/LLTrack.h"
#include "commands/SetArmAwait.h"
#include "commands/SetIntakeAwait.h"
#include "commands/SetRPMAwait.h"
#include "subsystems/ArmSubsystem.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/ShooterSubsystem.h"

namespace Commands {
  frc2::CommandPtr Handoff(ArmSubsystem* arm, IntakeSubsystem* intake,
                           ShooterSubsystem* shooter) {
    return frc2::cmd::Sequence(
        SetArmAwait(arm, ArmSubsystem::State::kHandoff).ToPtr(),
        SetIntakeAwait(intake, IntakeSubsystem::PivotState::kHandoff).ToPtr(),
        intake->SetIntakeTargetCMD(IntakeSubsystem::IntakeState::kHandoff),
        shooter->SetFeederCMD(-0.5), frc2::cmd::Wait(1_s),
        shooter->SetFeederCMD(0),
        intake->SetIntakeTargetCMD(IntakeSubsystem::IntakeState::kStopped),
        SetIntakeAwait(intake, IntakeSubsystem::PivotState::kStow)
            .Until([intake] { return intake->GetAngle() > -50_deg; }),
        shooter->SetFeederCMD(0.1), frc2::cmd::Wait(0.075_s),
        shooter->SetFeederCMD(0.0));
  }

  frc2::CommandPtr ShootNote(ShooterSubsystem* shooter) {
    return frc2::cmd::Sequence(
        shooter->SetFeederCMD(-1), frc2::cmd::Wait(1_s),
        shooter->SetFeederCMD(0),
        shooter->SetTargetStateCMD(ShooterSubsystem::State::kStopped));
  }

  frc2::CommandPtr RevShooter(ShooterSubsystem* shooter) {
    return SetRPMAwait(shooter, ShooterSubsystem::State::kSpeaker)
        .ToPtr()
        .AndThen(frc2::cmd::Wait(0.5_s));
  }
}  // namespace Commands
