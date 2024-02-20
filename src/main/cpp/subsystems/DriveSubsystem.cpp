// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveSubsystem.h"

#include <frc/DriverStation.h>
#include <frc/filter/MedianFilter.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>
#include <wpi/sendable/SendableBuilder.h>

#include "Constants.h"
#include "utils/Util.h"

using namespace DriveConstants;
using namespace DriveConstants::CanIds;
using namespace pathplanner;

DriveSubsystem::DriveSubsystem()
    : m_frontLeft{kFrontLeftDriveMotorPort, kFrontLeftTurningMotorPort,
                  kFrontLeftTurningEncoderPorts, kFrontLeftOffset},

      m_rearLeft{kRearLeftDriveMotorPort, kRearLeftTurningMotorPort,
                 kRearLeftTurningEncoderPorts, kRearLeftOffset},

      m_frontRight{kFrontRightDriveMotorPort, kFrontRightTurningMotorPort,
                   kFrontRightTurningEncoderPorts, kFrontRightOffset},

      m_rearRight{kRearRightDriveMotorPort, kRearRightTurningMotorPort,
                  kRearRightTurningEncoderPorts, kRearRightOffset},

      m_gyro(DriveConstants::CanIds::kPidgeonID),
      m_visionSystem(VisionSubsystem::GetInstance()),
      m_poseEstimator(kDriveKinematics, GetHeading(), GetModulePositions(),
                      frc::Pose2d()),
      m_vision(true) {
  frc::SmartDashboard::PutData("Field", &m_field);
  AutoBuilder::configureHolonomic(
      [this] { return GetPose(); }, [this](frc::Pose2d pose) { SetPose(pose); },
      [this] { return GetVelocity(); },
      [this](frc::ChassisSpeeds speeds) { DriveRobotRelative(speeds); },
      AutoConstants::kConfig,
      [this] {
        auto alliance = frc::DriverStation::GetAlliance();
        if (alliance) {
          return alliance.value() == frc::DriverStation::Alliance::kRed;
        }
        return false;
      },
      this);
}

void DriveSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
  m_poseEstimator.Update(GetHeading(), GetModulePositions());

  // if (m_vision) {
  //   std::vector<PosePacket> CamPose = m_visionSystem.GetPose();
  //   for (PosePacket i : CamPose) {
  //     m_poseEstimator.AddVisionMeasurement(i.pose, i.timestamp);
  //   }
  // }

  m_field.SetRobotPose(m_poseEstimator.GetEstimatedPosition());
}

void DriveSubsystem::Drive(units::meters_per_second_t xSpeed,
                           units::meters_per_second_t ySpeed,
                           units::radians_per_second_t rot,
                           bool fieldRelative) {
  auto states = kDriveKinematics.ToSwerveModuleStates(
      fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                          xSpeed, ySpeed, rot, GetHeading())
                    : frc::ChassisSpeeds{xSpeed, ySpeed, rot});

  SetModuleStates(states);
}

void DriveSubsystem::DriveRobotRelative(frc::ChassisSpeeds speeds) {
  Drive(speeds.vx, speeds.vy, speeds.omega, false);
}

void DriveSubsystem::SetModuleStates(
    wpi::array<frc::SwerveModuleState, 4> desiredStates) {
  kDriveKinematics.DesaturateWheelSpeeds(&desiredStates,
                                         ModuleConstants::kMaxModuleSpeed);

  m_frontLeft.SetDesiredState(desiredStates[0]);
  m_frontRight.SetDesiredState(desiredStates[1]);
  m_rearLeft.SetDesiredState(desiredStates[2]);
  m_rearRight.SetDesiredState(desiredStates[3]);
}

frc::ChassisSpeeds DriveSubsystem::GetVelocity() const {
  return kDriveKinematics.ToChassisSpeeds(GetModuleStates());
}

wpi::array<frc::SwerveModuleState, 4> DriveSubsystem::GetModuleStates() const {
  return {m_frontLeft.GetState(), m_frontRight.GetState(),
          m_rearLeft.GetState(), m_rearRight.GetState()};
}

wpi::array<frc::SwerveModulePosition, 4> DriveSubsystem::GetModulePositions()
    const {
  return {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
          m_rearLeft.GetPosition(), m_rearRight.GetPosition()};
}

frc::Rotation2d DriveSubsystem::GetHeading() const {
  return m_gyro.GetRot2d().Degrees() + m_offset;
}

void DriveSubsystem::SetOffset(units::degree_t offset) {
  m_offset = offset;
}

units::degree_t DriveSubsystem::GetRoll() const {
  return m_gyro.GetRoll();
}

units::degree_t DriveSubsystem::GetPitch() const {
  return m_gyro.GetPitch();
}

frc::Pose2d DriveSubsystem::GetPose() const {
  return m_poseEstimator.GetEstimatedPosition();
}

void DriveSubsystem::SetPose(frc::Pose2d pose) {
  m_poseEstimator.ResetPosition(GetHeading(), GetModulePositions(), pose);
}

frc2::CommandPtr DriveSubsystem::SetGyro(units::degree_t angle) {
  return this->RunOnce([angle, this] { SetOffset(angle); });
}

void DriveSubsystem::InitSendable(wpi::SendableBuilder& builder) {
#define LAMBDA(x) [this] { return x; }

  builder.SetSmartDashboardType("Swerve Drive");

  builder.AddBooleanProperty("Vision", LAMBDA(m_vision),
                             [this](bool set) -> void { m_vision = set; });

  builder.AddDoubleProperty("X Velocity", LAMBDA(GetVelocity().vx.value()),
                            nullptr);
  builder.AddDoubleProperty("Y Velocity", LAMBDA(GetVelocity().vy.value()),
                            nullptr);
  builder.AddDoubleProperty("Rotation Velocity",
                            LAMBDA(GetVelocity().omega.value()), nullptr);
  builder.AddDoubleProperty(
      "Velocity",
      LAMBDA(hb::hypot(GetVelocity().vx.value(), GetVelocity().vy.value())),
      nullptr);

  builder.AddDoubleProperty("Rear Right Angle",
                            LAMBDA(m_rearRight.GetCoderAngle()), nullptr);

#undef LAMBDA
}
