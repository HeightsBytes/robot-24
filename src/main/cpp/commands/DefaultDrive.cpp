// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DefaultDrive.h"

#include <frc/MathUtil.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <utility>

#include "Constants.h"
#include "units/velocity.h"
#include "utils/Util.h"

DefaultDrive::DefaultDrive(DriveSubsystem* drive, std::function<double()> leftY,
                           std::function<double()> leftX,
                           std::function<double()> rightX,
                           std::function<double()> triggerAxis)
    : m_drive(drive),
      m_leftY(std::move(leftY)),
      m_leftX(std::move(leftX)),
      m_rightX(std::move(rightX)),
      m_triggerAxis(std::move(triggerAxis)) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(drive);
}

// Called when the command is initially scheduled.
void DefaultDrive::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void DefaultDrive::Execute() {
  // when the trigger is held down, the speed is decreased to a minimum of 3/8 speed
  double maxSpeed = DriveConstants::kMaxChassisSpeed.value() *
                    (1 - m_triggerAxis() * 0.625);

  // Note: x is forwards, y is side to side.
  // This means 'x' is the traditional y direction
  // 'y' is the tradtional x
  double x = -m_leftX();
  double y = m_leftY();
  double rotationMagnitude = -frc::ApplyDeadband(m_rightX(), 0.03);

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
  units::radians_per_second_t rotation = units::radians_per_second_t(
      rotationMagnitude * DriveConstants::kMaxAngularSpeed.value());

  m_drive->Drive(xComponent, yComponent, rotation, true);
}

// Called once the command ends or is interrupted.
void DefaultDrive::End(bool interrupted) {}

// Returns true when the command should end.
bool DefaultDrive::IsFinished() {
  return false;
}
