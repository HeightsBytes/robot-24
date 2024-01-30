// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/geometry/Transform3d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <pathplanner/lib/path/PathConstraints.h>
#include <pathplanner/lib/util/HolonomicPathFollowerConfig.h>
#include <pathplanner/lib/util/PIDConstants.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include <numbers>

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or bool constants.  This should not be used for any other purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace DriveConstants {

  namespace CanIds {

    inline constexpr int kFrontLeftDriveMotorPort = 1;
    inline constexpr int kRearLeftDriveMotorPort = 3;
    inline constexpr int kFrontRightDriveMotorPort = 6;
    inline constexpr int kRearRightDriveMotorPort = 8;

    inline constexpr int kFrontLeftTurningMotorPort = 5;
    inline constexpr int kRearLeftTurningMotorPort = 7;
    inline constexpr int kFrontRightTurningMotorPort = 2;
    inline constexpr int kRearRightTurningMotorPort = 4;

    inline constexpr int kFrontLeftTurningEncoderPorts = 2;
    inline constexpr int kRearLeftTurningEncoderPorts = 1;
    inline constexpr int kFrontRightTurningEncoderPorts = 3;
    inline constexpr int kRearRightTurningEncoderPorts = 4;

    inline constexpr int kPidgeonID = 0;

  }  // namespace CanIds

  inline constexpr double kFrontLeftOffset = 41.31 - 180.0;    // encoder 2
  inline constexpr double kRearLeftOffset = -42.71 + 180.0;    // encoder 1
  inline constexpr double kFrontRightOffset = 107.48 - 180.0;  // encoder 3
  inline constexpr double kRearRightOffset = -168.75 + 180.0;  // encoder 4

  inline constexpr auto kMaxChassisSpeed = 4.25_mps;
  inline constexpr auto kMaxAngularSpeed =
      units::radians_per_second_t(1 * std::numbers::pi);
  inline constexpr auto kMaxAngularAcceleration =
      units::radians_per_second_squared_t(2 * std::numbers::pi);

  inline constexpr auto kTrackWidth = 12_in;   // half
  inline constexpr auto kTrackLength = 15_in;  // half

  // Forward is +x and left is +y
  inline frc::SwerveDriveKinematics<4> kDriveKinematics{
      frc::Translation2d(kTrackLength, kTrackWidth),
      frc::Translation2d(kTrackLength, -kTrackWidth),
      frc::Translation2d(-kTrackLength, kTrackWidth),
      frc::Translation2d(-kTrackLength, -kTrackWidth)};

}  // namespace DriveConstants

namespace ModuleConstants {

  inline constexpr double kGearRatio = 1 / 6.75;
  inline constexpr double kWheelDiameterMeters = 0.05092958;
  inline constexpr double kDriveEncoderDistancePerPulse =
      kGearRatio * 2 * std::numbers::pi * kWheelDiameterMeters;
  inline constexpr double kDriveEncoderVelocityRatio =
      kDriveEncoderDistancePerPulse;
  inline constexpr double kDriveEncoderPositionRatio =
      kDriveEncoderDistancePerPulse;

  inline constexpr double kTurnRatio = 7.0 / 150.0;
  inline constexpr double kTurnEncoderRatio =
      kTurnRatio * 2.0 * std::numbers::pi;

  inline constexpr double kPModuleTurningController = 1;
  inline constexpr double kPModuleDriveController = 0.75;

  inline constexpr double kPDrive = 0.175;
  inline constexpr double kIDrive = 0;
  inline constexpr double kDDrive = 0.02;
  inline constexpr double kFFDrive = 2.67;

  inline constexpr double kPTurn = 1.25;
  inline constexpr double kITurn = 0;
  inline constexpr double kDTurn = 0;
  inline constexpr double kFFTurn = 0;

  inline constexpr auto kMaxModuleSpeed = 4.5_mps;

}  // namespace ModuleConstants

namespace VisionConstants {
  inline const frc::Transform3d RightTransform{
      frc::Translation3d(-15_in, -7_in, 24_in),
      frc::Rotation3d{0_deg, 0_deg, -150_deg}};
  inline const frc::Transform3d LeftTransform{
      frc::Translation3d(-15_in, 7_in, 24_in),
      frc::Rotation3d{0_deg, 0_deg, 150_deg}};
}  // namespace VisionConstants

namespace IntakeConstants {
  // TODO: find constants

  inline constexpr int kPivotMotorID = 0;
  inline constexpr int kIntakeMotorID = 0;

  inline constexpr double kP = 0;
  inline constexpr double kI = 0;
  inline constexpr double kD = 0;

  inline constexpr int kLimitID = 0;

  inline constexpr double kPositionConversion = 360;

  inline constexpr auto kIntakeLimit = 0_A;
  inline constexpr auto kPivotLimit = 0_A;

  namespace Positions {
    inline constexpr auto kGround = 0_deg;
    inline constexpr auto kStow = 0_deg;
  }  // namespace Positions

  namespace Speeds {
    inline constexpr double kEject = 0;
    inline constexpr double kIntake = 0;
    inline constexpr double kFeedShooter = 0;
  }  // namespace Speeds

}  // namespace IntakeConstants

namespace ClimbConstants {
  // TODO: find constants

  inline constexpr int kMotorLeftID = 0;
  inline constexpr int kMotorRightID = 0;

  inline constexpr double kP = 0;
  inline constexpr double kI = 0;
  inline constexpr double kD = 0;
  inline constexpr double kFF = 0;

  inline constexpr double kPositionConversion = 1;

  inline constexpr double kMinPower = -1;
  inline constexpr double kMaxPower = 1;

  inline constexpr auto kZeroingCurrentThreshold = 0_A;

  inline constexpr auto kPositionTollerance = 0.05_m;

  namespace Positions {
    inline constexpr auto kStow = 0_m;
    inline constexpr auto kMax = 0_m;
    inline constexpr auto kTrap = 0_m;
  }  // namespace Positions

}  // namespace ClimbConstants

namespace ArmConstants {
  // TODO: find constants

  inline constexpr int kMotorID = 0;

  inline constexpr double kP = 0;
  inline constexpr double kI = 0;
  inline constexpr double kD = 0;
  inline constexpr double kG = 0;

  inline constexpr auto kOffset = 0_deg;

  inline constexpr auto kCurrentLimit = 0_A;

  inline constexpr double kConversionFactor = 360;

  inline constexpr bool kEncoderInverted = false;

  namespace Setpoint {
    inline constexpr auto kStow = 0_deg;
    inline constexpr auto kAmp = 0_deg;
    inline constexpr auto kTollerance = 1_deg;
  }  // namespace Setpoint

}  // namespace ArmConstants

namespace ShooterConstants {
  // TODO: find constants

  inline constexpr int kLeftFlywheelID = 0;
  inline constexpr int kRightFlywheelID = 0;
  inline constexpr int kLeftFeederID = 0;
  inline constexpr int kRightFeederID = 0;
  inline constexpr int kBeamBreakPort = 0;

  inline constexpr double kP = 0;
  inline constexpr double kI = 0;
  inline constexpr double kD = 0;
  inline constexpr double kFF = 0;

  inline constexpr auto kCurrentLimit = 0_A;

  namespace Setpoint {
    inline constexpr auto kMax = 5880_rpm;
    inline constexpr auto kIdle = 300_rpm;
    inline constexpr auto kShooting = 5000_rpm;  // guesstimate
    inline constexpr auto kTrapAmp = 2000_rpm;   // also guesstimate
  }                                              // namespace Setpoint
  inline constexpr auto kTollerance = 100_rpm;

}  // namespace ShooterConstants

namespace AutoConstants {

  inline constexpr auto kMaxSpeed = 3_mps;
  inline constexpr auto kMaxAcceleration = 3_mps_sq;
  inline constexpr auto kMaxAngularSpeed =
      units::radians_per_second_t(std::numbers::pi);
  inline constexpr auto kMaxAngularAcceleration =
      units::radians_per_second_squared_t(std::numbers::pi);

  inline constexpr pathplanner::PIDConstants kPIDTranslation{1.25, 0, 0.07};
  inline constexpr pathplanner::PIDConstants kPIDRotation{1, 0, 0.1};

  inline constexpr pathplanner::HolonomicPathFollowerConfig kConfig{
      kPIDTranslation, kPIDRotation, kMaxSpeed,
      0.53881_m, /**std::sqrt(2 * 15in ^ 2)**/
      pathplanner::ReplanningConfig()};

  inline constexpr pathplanner::PathConstraints kConstraints{
      kMaxSpeed, kMaxAcceleration, kMaxAngularSpeed, kMaxAngularAcceleration};

}  // namespace AutoConstants

namespace OIConstants {
  inline constexpr int kDriverControllerPort = 0;
  inline constexpr int kOperatorControllerPort = 1;
}  // namespace OIConstants
