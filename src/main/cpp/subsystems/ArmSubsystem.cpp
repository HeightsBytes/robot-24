// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ArmSubsystem.h"

#include <frc/DriverStation.h>
#include <frc/MathUtil.h>
#include <units/math.h>

#include <utility>

#include "Constants.h"

ArmSubsystem::ArmSubsystem(std::function<frc::Pose2d()> poseFunction)
    : m_motor(ArmConstants::kMotorID, rev::CANSparkMax::MotorType::kBrushless),
      m_encoder(m_motor.GetAbsoluteEncoder(
          rev::SparkAbsoluteEncoder::Type::kDutyCycle)),
      m_controller(m_motor.GetPIDController()),
      m_vision(VisionSubsystem::GetInstance()),
      m_actual(State::kSwitching),
      m_target(State::kStow),
      m_pose(std::move(poseFunction)),
      m_angleFunction(0, 0) {
  m_motor.RestoreFactoryDefaults();

  m_motor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_motor.SetSmartCurrentLimit(ArmConstants::kCurrentLimit.value());

  m_encoder.SetPositionConversionFactor(ArmConstants::kConversionFactor);
  m_encoder.SetInverted(ArmConstants::kEncoderInverted);
  m_encoder.SetZeroOffset(ArmConstants::kOffset.value());

  m_controller.SetFeedbackDevice(m_encoder);
  m_controller.SetP(ArmConstants::kP);
  m_controller.SetI(ArmConstants::kI);
  m_controller.SetD(ArmConstants::kD);

  m_motor.BurnFlash();
}

// This method will be called once per scheduler run
void ArmSubsystem::Periodic() {
  CheckState();
  ControlLoop();
}

units::degree_t ArmSubsystem::GetAngle() const {
  return units::degree_t(m_encoder.GetPosition());
}

bool ArmSubsystem::AtTarget() const {
    return GetCurrentState() == GetTargetState();
}

ArmSubsystem::State ArmSubsystem::GetCurrentState() const {
    return m_actual;
}

ArmSubsystem::State ArmSubsystem::GetTargetState() const {
    return m_target;
}

void ArmSubsystem::SetState(State state) {
    m_target = state;
}

frc2::CommandPtr ArmSubsystem::SetStateCMD(State state) {
    return this->RunOnce([this, state] {SetState(state);});
}

void ArmSubsystem::InitSendable(wpi::SendableBuilder& builder) {
  builder.SetSmartDashboardType("Arm");

#define LAMBDA(x) [this] { return x; }

  builder.AddStringProperty("Actual State", LAMBDA(ToStr(m_actual)), nullptr);
  builder.AddStringProperty("Target State", LAMBDA(ToStr(m_target)), nullptr);

  builder.AddDoubleProperty("Angle", LAMBDA(GetAngle().value()), nullptr);

  builder.AddBooleanProperty("At Target", LAMBDA(AtTarget()), nullptr);

#undef LAMBDA
}

void ArmSubsystem::ControlLoop() {
  m_controller.SetReference(ToAngle(m_target).value(),
                            rev::CANSparkMax::ControlType::kPosition, 0,
                            CalculateKg().value());
}

void ArmSubsystem::CheckState() {
  auto angle = GetAngle();

  // Check Stow
  if (frc::IsNear(ArmConstants::Setpoint::kStow, angle,
                  ArmConstants::Setpoint::kTollerance)) {
    m_actual = State::kStow;
    return;
  }

  // Check Amp
  if (frc::IsNear(ArmConstants::Setpoint::kAmp, angle,
                  ArmConstants::Setpoint::kTollerance)) {
    m_actual = State::kStow;
    return;
  }

  // Check Targetting
  if (frc::IsNear(StageAngle(), angle, ArmConstants::Setpoint::kTollerance)) {
    m_actual = State::kTargeting;
    return;
  }

  m_actual = State::kSwitching;
}

units::degree_t ArmSubsystem::ToAngle(State state) const {
  switch (state) {
    default:
    case State::kSwitching:
      return GetAngle();
      break;

    case State::kAmp:
      return ArmConstants::Setpoint::kAmp;
      break;

    case State::kStow:
      return ArmConstants::Setpoint::kStow;
      break;

    case State::kTargeting:
      return StageAngle();
      break;
  }
}

units::degree_t ArmSubsystem::StageAngle() const {
  std::optional<units::degree_t> llangle = m_vision.AngleToStage();
  if (llangle) {
    return llangle.value();
  } else if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue) {
    return units::degree_t(m_angleFunction.Calculate(
        m_pose()
            .Translation()
            .Distance(m_vision.GetTagPose(7)->ToPose2d().Translation())
            .value()));
  } else {
    return units::degree_t(m_angleFunction.Calculate(
        m_pose()
            .Translation()
            .Distance(m_vision.GetTagPose(4)->ToPose2d().Translation())
            .value()));
  }
}

units::volt_t ArmSubsystem::CalculateKg() const {
  auto ret = ArmConstants::kG * units::math::cos(GetAngle());

  return units::volt_t(ret.value());
}

std::string ArmSubsystem::ToStr(State state) const {
  switch (state) {
    default:
    case State::kSwitching:
      return "Switching";
      break;

    case State::kStow:
      return "Stow";
      break;

    case State::kAmp:
      return "Amp";
      break;

    case State::kTargeting:
      return "Targeting";
      break;
  }
}
