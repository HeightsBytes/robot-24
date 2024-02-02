// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RequestSpeaker.h"

#include <frc/DriverStation.h>
#include <frc/MathUtil.h>

#include <utility>

#include "Constants.h"
#include "utils/Util.h"
#include "utils/cams/Limelight.h"

RequestSpeaker::RequestSpeaker(ArmSubsystem* arm, DriveSubsystem* drive,
                               ShooterSubsystem* shooter,
                               std::function<double()> leftX,
                               std::function<double()> leftY)
    : m_arm(arm),
      m_drive(drive),
      m_shooter(shooter),
      m_vision(VisionSubsystem::GetInstance()),
      m_controller(AutoConstants::kPIDRotation.kP,
                   AutoConstants::kPIDRotation.kI,
                   AutoConstants::kPIDRotation.kD),
      m_leftX(std::move(leftX)),
      m_leftY(std::move(leftY)) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({arm, drive, shooter});
}

// Called when the command is initially scheduled.
void RequestSpeaker::Initialize() {
  m_arm->SetState(ArmSubsystem::State::kTargeting);
  m_shooter->SetTargetState(ShooterSubsystem::State::kSpeaker);
}

// Called repeatedly when this Command is scheduled to run
void RequestSpeaker::Execute() {
  double rotOut;

  int AprilID =
      frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue
          ? ShooterConstants::kBlueApriltagID
          : ShooterConstants::kRedApriltagID;

  std::optional result1 = VisionSubsystem::HasResult(
      m_vision.GetLeftResult().GetTargets(), AprilID);
  std::optional result2 = VisionSubsystem::HasResult(
      m_vision.GetRightResult().GetTargets(), AprilID);

  frc::Pose2d pose = m_drive->GetPose();

  if (result1) {
    rotOut = m_controller.Calculate(
        m_drive->GetHeading().Degrees().value(),
        result1.value().GetYaw() - VisionConstants::LeftTransform.Rotation()
                                       .ToRotation2d()
                                       .Degrees()
                                       .value());
  } else if (result2) {
    rotOut = m_controller.Calculate(
        m_drive->GetHeading().Degrees().value(),
        result2.value().GetYaw() - VisionConstants::RightTransform.Rotation()
                                       .ToRotation2d()
                                       .Degrees()
                                       .value());
  } else {
    rotOut = m_controller.Calculate(
        m_drive->GetHeading().Degrees().value(),
        pose.RelativeTo(m_vision.GetTagPose(AprilID)->ToPose2d())
            .Rotation()
            .Degrees()
            .value());
  }

  // Default Driving Code Below:

  double maxSpeed = DriveConstants::kMaxChassisSpeed.value();

  // Note: x is forwards, y is side to side.
  // This means 'x' is the traditional y direction
  // 'y' is the tradtional x
  double x = -m_leftX();
  double y = m_leftY();

  double magnitude =
      std::pow(frc::ApplyDeadband(hb::hypot(x, y), 0.01), 2) * maxSpeed;

  // Determining the angle itself. If y==0 then we can simply multiply pi/2 by
  // the sign of x
  double angle = y == 0 ? hb::sgn(x) * std::numbers::pi / 2 : std::atan(x / y);
  // Below we have to consider quadrants. Because arctan is limited to -pi/2 to
  // pi/2 Check second quadrant
  if (x > 0 && y < 0)
    angle += std::numbers::pi;
  // Check third quadrant
  if (x < 0 && y < 0)
    angle += std::numbers::pi;
  // Check edge case where x is zero and y is across zero
  if (x == 0 && y < 0)
    angle += std::numbers::pi;

  units::meters_per_second_t xComponent =
      units::meters_per_second_t(magnitude * std::sin(angle));
  units::meters_per_second_t yComponent =
      -units::meters_per_second_t(magnitude * std::cos(angle));

  m_drive->Drive(xComponent, yComponent, units::radians_per_second_t(rotOut),
                 true);
}

// Called once the command ends or is interrupted.
void RequestSpeaker::End(bool interrupted) {}

// Returns true when the command should end.
bool RequestSpeaker::IsFinished() {
  return false;
}
