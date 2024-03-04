// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Encoder.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/motorcontrol/Spark.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <rev/CANSparkMax.h>
#include <units/length.h>
#include <units/temperature.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include "utils/Util.h"

#include "utils/swerve/CANCoder.h"

class SwerveModule {
 public:
  SwerveModule(int driveMotorChannel, int turningMotorChannel,
               const int turningEncoderPorts, const double offset);
  frc::SwerveModuleState GetState() const;

  frc::SwerveModulePosition GetPosition() const;

  double GetCoderAngle() { return m_turningEncoder.Get().value(); }

  void SetDesiredState(const frc::SwerveModuleState& state);

  void ResetEncoders();

  void ZeroTurnEncoder();

  void StopMotors();

  double turnOut() {
    return m_turningMotor.GetAppliedOutput();
  }

 private:
  rev::CANSparkMax m_driveMotor;
  rev::CANSparkMax m_turningMotor;

  rev::SparkRelativeEncoder m_sparkDriveEncoder;
  rev::SparkRelativeEncoder m_sparkTurnEncoder;

  rev::SparkPIDController m_tController;
  rev::SparkPIDController m_dController;

  hb::S_CANCoder m_turningEncoder;

  int m_id;
};
