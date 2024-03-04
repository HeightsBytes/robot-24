#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>

#include "subsystems/DriveSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/ArmSubsystem.h"
#include "subsystems/ShooterSubsystem.h"

#include "commands/DefaultDrive.h"
#include "commands/DriveAndTrack.h"
#include "commands/LLTrack.h"
#include "commands/SetArmAwait.h"
#include "commands/SetIntakeAwait.h"
#include "commands/SetRPMAwait.h"

namespace Commands {
  frc2::CommandPtr Handoff(ArmSubsystem* arm, IntakeSubsystem* intake, ShooterSubsystem* shooter) {
    return frc2::cmd::Sequence(
    SetArmAwait(arm, ArmSubsystem::State::kHandoff).ToPtr(),
    SetIntakeAwait(intake, IntakeSubsystem::PivotState::kHandoff).ToPtr(),
    frc2::cmd::Wait(0.5_s),
    intake->SetIntakeTargetCMD(IntakeSubsystem::IntakeState::kHandoff),
    shooter->SetFeederCMD(-0.5),
    frc2::cmd::Wait(1_s),
    shooter->SetFeederCMD(0),
    intake->SetIntakeTargetCMD(IntakeSubsystem::IntakeState::kStopped),
    SetIntakeAwait(intake, IntakeSubsystem::PivotState::kStow).ToPtr(),
    shooter->SetFeederCMD(0.1),
    frc2::cmd::Wait(0.1_s),
    shooter->SetFeederCMD(0.0),
    arm->SetTargetStateCMD(ArmSubsystem::State::kStow)
    );
  }

  frc2::CommandPtr ShootNote(ShooterSubsystem* shooter) {
    return frc2::cmd::Sequence(
      shooter->SetFeederCMD(-1),
      frc2::cmd::Wait(0.5_s),
      shooter->SetFeederCMD(0),
      shooter->SetTargetStateCMD(ShooterSubsystem::State::kStopped));
  }

  frc2::CommandPtr RevShooter(ShooterSubsystem* shooter) {
    return SetRPMAwait(shooter, ShooterSubsystem::State::kSpeaker).ToPtr().WithTimeout(1.75_s);
  }
  
}