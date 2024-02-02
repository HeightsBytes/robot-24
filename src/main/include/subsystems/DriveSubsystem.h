// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc2/command/SubsystemBase.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/util/HolonomicPathFollowerConfig.h>
#include <pathplanner/lib/util/PIDConstants.h>
#include <pathplanner/lib/util/ReplanningConfig.h>
#include <units/angle.h>
#include <wpi/array.h>

#include "Constants.h"
#include "frc/filter/SlewRateLimiter.h"
#include "subsystems/VisionSubsystem.h"
#include "units/velocity.h"
#include "utils/swerve/PigeonGyro.h"
#include "utils/swerve/SwerveModule.h"

class DriveSubsystem : public frc2::SubsystemBase {
 public:
  DriveSubsystem();

  void Periodic() override;

  /**
   * Drives the robot at given x, y and theta speeds. Speeds range from [-1, 1]
   * and the linear speeds have no effect on the angular speed.
   *
   * @param xSpeed        Speed of the robot in the x direction
   *                      (forward/backwards).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to
   *                      the field.
   */
  void Drive(units::meters_per_second_t xSpeed,
             units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
             bool fieldRelative);

  void DriveRobotRelative(frc::ChassisSpeeds speeds);

  /**
   * Sets the drive MotorControllers to a power from -1 to 1.
   */
  void SetModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates);

  frc::ChassisSpeeds GetVelocity() const;

  /**
   * @returns array of the modules speeds and directions in the order [fl, fr,
   * rl, rr]
   */
  wpi::array<frc::SwerveModuleState, 4> GetModuleStates() const;

  /**
   * @returns array of the modules displacements and directions in the order
   * [fl, fr, rl, rr]
   */
  wpi::array<frc::SwerveModulePosition, 4> GetModulePositions() const;

  /**
   * @brief Returns the offseted heading
   *
   * @return frc::Rotation2d
   */
  frc::Rotation2d GetHeading() const;

  /**
   * @brief Sets the offset of the gyro
   *
   * @param offset
   */
  void SetOffset(units::degree_t offset);

  /**
   * @brief Returns the roll of the robot
   *
   * @return double
   */
  units::degree_t GetRoll() const;

  /**
   * @brief Returns the pitch of the robot
   *
   * @return double
   */
  units::degree_t GetPitch() const;

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  frc::Pose2d GetPose() const;

  // Sets the pose of the robot's estimator
  void SetPose(frc::Pose2d pose);

  [[nodiscard]]
  frc2::CommandPtr SetGyro(units::degree_t angle);

  void InitSendable(wpi::SendableBuilder& builder) override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  SwerveModule m_frontLeft;
  SwerveModule m_rearLeft;
  SwerveModule m_frontRight;
  SwerveModule m_rearRight;

  hb::PigeonGyro m_gyro;

  units::degree_t m_offset = 0_deg;

  frc::Field2d m_field;

  VisionSubsystem& m_visionSystem;

  frc::SwerveDrivePoseEstimator<4> m_poseEstimator;

  bool m_vision;
};
