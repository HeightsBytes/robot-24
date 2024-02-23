// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DriveAndTrack.h"

#include <frc/DriverStation.h>
#include <frc/geometry/Transform2d.h>

#include <cmath>

#include "Constants.h"
#include "utils/Util.h"

DriveAndTrack::DriveAndTrack(DriveSubsystem* drive,
                             std::function<double()> leftY,
                             std::function<double()> leftX)
    : m_drive(drive),
      m_vision(VisionSubsystem::GetInstance()),
      m_leftY(std::move(leftY)),
      m_leftX(std::move(leftX)) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(drive);
}

// Called when the command is initially scheduled.
void DriveAndTrack::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void DriveAndTrack::Execute() {
  auto color = frc::DriverStation::GetAlliance();
  int id;
  if (!color) {
    std::printf("[WARNING] DriveAndTrack Command failed to execute\n");
    return;
  }
  if (color.value() == frc::DriverStation::Alliance::kBlue) {
    id = ShooterConstants::kBlueApriltagID;
  } else {
    id = ShooterConstants::kRedApriltagID;
  }

  frc::Pose2d targetPose = m_vision.GetTagPose(id)->ToPose2d();
  frc::Pose2d currentPose = m_drive->GetPose();

  auto target = currentPose.RelativeTo(targetPose);

  double maxSpeed = DriveConstants::kMaxChassisSpeed.value();
  double x = -m_leftY();
  double y = m_leftX();
  double magnitude =
      std::pow(frc::ApplyDeadband(std::clamp(hb::hypot(x, y), 0.0, 1.0), 0.01),
               2) *
      maxSpeed;

  double angle = y == 0 ? hb::sgn(x) * std::numbers::pi / 2 : std::atan(x / y);

  if (y < 0)
    angle += std::numbers::pi;

  auto xComponent = units::meters_per_second_t(magnitude * std::sin(angle));
  auto yComponent = units::meters_per_second_t(magnitude * std::cos(angle));

  double rotationOut = m_controller.Calculate(
      m_drive->GetHeading().Degrees().value(),
      target.Rotation().RotateBy(180_deg).Degrees().value());

  m_drive->Drive(xComponent, yComponent,
                 units::radians_per_second_t(rotationOut), true);
}

// Called once the command ends or is interrupted.
void DriveAndTrack::End(bool interrupted) {}

// Returns true when the command should end.
bool DriveAndTrack::IsFinished() {
  return false;
}
