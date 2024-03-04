// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/LLTrack.h"

#include <utility>

#include "utils/cams/Limelight.h"

LLTrack::LLTrack(DriveSubsystem* drive, std::function<double()> leftY,
                 std::function<double()> leftX)
    : m_drive(drive), m_leftY(std::move(leftY)), m_leftX(std::move(leftX)) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(drive);
  m_angle = 0_deg;
  m_hasTarget = false;
  m_controller.EnableContinuousInput(-180.0, 180.0);
}

// Called when the command is initially scheduled.
void LLTrack::Initialize() {
  m_hasTarget = hb::LimeLight::HasTarget();
  m_angle = units::degree_t(hb::LimeLight::GetX());
}

// Called repeatedly when this Command is scheduled to run
void LLTrack::Execute() {
  double maxSpeed = 1.5;
  double x = -m_leftY();
  double y = m_leftX();

  double magnitude =
      std::pow(frc::ApplyDeadband(std::clamp(hb::hypot(x, y), 0.0, 1.0), 0.1),
               2) *
      maxSpeed;

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

  if (hb::LimeLight::HasTarget()) {
    m_angle = units::degree_t(hb::LimeLight::GetX());
  }

  m_drive->Drive(
      xComponent, yComponent,
      units::radians_per_second_t(m_controller.Calculate(m_angle.value(), 0)),
      true);
}

// Called once the command ends or is interrupted.
void LLTrack::End(bool interrupted) {}

// Returns true when the command should end.
bool LLTrack::IsFinished() {
  if (!m_hasTarget)
    return true;
  return hb::InRange(m_angle.value(),
                     m_drive->GetHeading().Degrees().value() - 180, 2);
}
