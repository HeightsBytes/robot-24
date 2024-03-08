// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "utils/cams/Limelight.h"

#include <frc/DriverStation.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

#include <optional>
#include <vector>

#include "utils/cams/PosePacket.h"

#define GETVAL(x)                        \
  nt::NetworkTableInstance::GetDefault() \
      .GetTable("limelight")             \
      ->GetNumber(x, 0.0)
#define GET_ARRAY_VAL(x, _size)          \
  nt::NetworkTableInstance::GetDefault() \
      .GetTable("limelight")             \
      ->GetNumberArray(x, std::span<const double>(_size))
#define SETVAL(x, y) \
  nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber(x, y)

using namespace hb;

bool LimeLight::HasTarget() {
  return static_cast<bool>(GETVAL("tv"));
}

double LimeLight::GetX() {
  return GETVAL("tx");
}

double LimeLight::GetY() {
  return GETVAL("ty");
}

double LimeLight::GetA() {
  return GETVAL("tx");
}

void LimeLight::SetLED(LEDMode Mode) {
  SETVAL("ledMode", (int)Mode);
}

void LimeLight::SetMode(CamMode Mode) {
  SETVAL("camMode", (int)Mode);
}

void LimeLight::SetPipeline(Pipeline Pipe) {
  SETVAL("pipeline", (int)Pipe);
}

LimeLight::Pipeline LimeLight::GetPipeline() {
  return LimeLight::Pipeline(GETVAL("pipeline"));
}

LimeLight::CamMode LimeLight::GetMode() {
  return LimeLight::CamMode(GETVAL("camMode"));
}

LimeLight::LEDMode LimeLight::GetLED() {
  return LimeLight::LEDMode(GETVAL("ledMode"));
}

// For this method, it appears that wpiblue should really be using wpiblue no
// matter what, will implment in this way
std::optional<PosePacket> LimeLight::GetPose() {
  static std::vector<double> results;
  if (!HasTarget()) {
    return std::nullopt;
  }
  results = nt::NetworkTableInstance::GetDefault()
                .GetTable("limelight")
                ->GetNumberArray("botpose_wpiblue", std::span<const double>());

  frc::Translation2d translation{units::meter_t(results[0]),
                                 units::meter_t(results[1])};
  frc::Rotation2d rotation{units::degree_t(results[5])};
  frc::Pose2d pose{translation, rotation};
  units::second_t timestamp{results[6]};

  return PosePacket(pose, timestamp);
}

int LimeLight::GetID() {
  return GETVAL("tid");
}