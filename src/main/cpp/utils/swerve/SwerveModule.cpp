// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "utils/swerve/SwerveModule.h"

#include <frc/geometry/Rotation2d.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <chrono>
#include <numbers>
#include <thread>

#include "Constants.h"

SwerveModule::SwerveModule(int driveMotorChannel, int turningMotorChannel,
                           const int turningEncoderPorts, const double offset)
    : m_driveMotor(driveMotorChannel, rev::CANSparkMax::MotorType::kBrushless),
      m_turningMotor(turningMotorChannel,
                     rev::CANSparkMax::MotorType::kBrushless),
      m_sparkDriveEncoder(m_driveMotor.GetEncoder(
          rev::SparkRelativeEncoder::Type::kHallSensor, 42)),
      m_sparkTurnEncoder(m_turningMotor.GetEncoder(
          rev::SparkRelativeEncoder::Type::kHallSensor, 42)),
      m_tController(m_turningMotor.GetPIDController()),
      m_dController(m_driveMotor.GetPIDController()),
      m_turningEncoder(turningEncoderPorts, offset),
      m_id(turningEncoderPorts) {
  m_driveMotor.RestoreFactoryDefaults();
  m_turningMotor.RestoreFactoryDefaults();

  m_driveMotor.SetSmartCurrentLimit(50);
  m_turningMotor.SetSmartCurrentLimit(20);

  // make motors default to break mode
  m_driveMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_turningMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  m_turningMotor.SetInverted(true);


  // set the turn conversion factors
  m_sparkTurnEncoder.SetPositionConversionFactor(
      ModuleConstants::kTurnEncoderRatio);

  m_sparkTurnEncoder.SetPosition(m_turningEncoder.Get().value());

  // set the drive conversion factor
  m_sparkDriveEncoder.SetVelocityConversionFactor(
      ModuleConstants::kDriveEncoderDistancePerPulse / 60);
  m_sparkDriveEncoder.SetPositionConversionFactor(
      ModuleConstants::kDriveEncoderDistancePerPulse);

  // init turning controller constants
  m_tController.SetPositionPIDWrappingEnabled(true);
  m_tController.SetPositionPIDWrappingMaxInput(std::numbers::pi);
  m_tController.SetPositionPIDWrappingMinInput(-std::numbers::pi);
  m_tController.SetP(ModuleConstants::kPTurn);
  m_tController.SetI(ModuleConstants::kITurn);
  m_tController.SetD(ModuleConstants::kDTurn);
  m_tController.SetFF(ModuleConstants::kFFTurn);
  m_tController.SetOutputRange(-1, 1);

  // init driving controller contants
  m_dController.SetP(ModuleConstants::kPDrive);
  m_dController.SetI(ModuleConstants::kIDrive);
  m_dController.SetD(ModuleConstants::kDDrive);
  m_dController.SetFF(ModuleConstants::kFFDrive * 1 / 12);
  m_dController.SetOutputRange(-1, 1);

  m_driveMotor.BurnFlash();
  m_turningMotor.BurnFlash();
}

frc::SwerveModuleState SwerveModule::GetState() const {
  return {units::meters_per_second_t{m_sparkDriveEncoder.GetVelocity()},
          frc::Rotation2d(units::radian_t(m_sparkTurnEncoder.GetPosition()))};
}

frc::SwerveModulePosition SwerveModule::GetPosition() const {
  return {units::meter_t(m_sparkDriveEncoder.GetPosition()),
          units::radian_t(m_sparkTurnEncoder.GetPosition())};
}

void SwerveModule::SetDesiredState(
    const frc::SwerveModuleState& referenceState) {
  // Optimize the reference state to avoid spinning further than 90 degrees
  const auto state = frc::SwerveModuleState::Optimize(
      referenceState, units::radian_t(m_sparkTurnEncoder.GetPosition()));

  if (std::fabs(state.speed.value()) < 0.01) {
    // Check to see if the input is very small, if it is, cancel all outputs
    StopMotors();
  } else {
    // If the outputs are sufficient, apply them with the PID Controllers
    m_dController.SetReference(state.speed.value(),
                               rev::CANSparkMax::ControlType::kVelocity);
    m_tController.SetReference(state.angle.Radians().value(),
                               rev::CANSparkMax::ControlType::kPosition);
  }
}

void SwerveModule::ResetEncoders() {
  m_sparkDriveEncoder.SetPosition(0);
  m_sparkTurnEncoder.SetPosition(m_turningEncoder.Get().value());
}

void SwerveModule::ZeroTurnEncoder() {
  // This is useful if you don't want to change the drive encoder reading
  m_sparkTurnEncoder.SetPosition(m_turningEncoder.Get().value());
}

void SwerveModule::StopMotors() {
  m_driveMotor.Set(0);
  m_turningMotor.Set(0);
}
