// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "utils/swerve/PigeonGyro.h"

#include <frc/Timer.h>

#include <cmath>
#include <numbers>

#include "utils/Util.h"

using namespace ctre::phoenix::sensors;
using namespace hb;

PigeonGyro::PigeonGyro(int ID) {
  pigeon = new ctre::phoenix::sensors::PigeonIMU(ID);
  pigeon->ConfigFactoryDefault();
  m_offset = 0;
}

units::degree_t PigeonGyro::GetAngle() const {
  if (pigeon->GetState() == PigeonIMU::Ready) {
    PigeonIMU::FusionStatus stat;
    pigeon->GetFusedHeading(stat);
    m_angle = stat.heading;
  }
  return units::degree_t(m_angle + m_offset);
}

units::degrees_per_second_t PigeonGyro::GetRate() const {
  if (pigeon->GetState() == PigeonIMU::Ready) {
    double rate[3];
    pigeon->GetRawGyro(rate);
    m_rate = rate[2];
  }
  return units::degrees_per_second_t(m_rate);
}

void PigeonGyro::Reset() {
  pigeon->SetFusedHeading(0, 30);
  m_angle = m_rate = 0;
}

units::degree_t PigeonGyro::GetPitch() const {
  return units::degree_t(pigeon->GetPitch());
}

units::degree_t PigeonGyro::GetRoll() const {
  return units::degree_t(pigeon->GetRoll());
}

frc::Rotation2d PigeonGyro::GetRot2d() const {
  return frc::Rotation2d(GetAngle());
}

units::radian_t PigeonGyro::GetRad() const {
  return units::radian_t((std::numbers::pi * GetAngle().value()) / 180);
}

void PigeonGyro::SetPosition(units::degree_t angle) {
  m_offset = angle.value();
}

units::degree_t PigeonGyro::GetCompassHeading() const {
  double angle = GetAngle().value();
  int initSign = hb::sgn(angle);

  // Check if the angle is already in range
  if (std::fabs(angle) < 180)
    return units::degree_t(angle);

  // add or subtract until the angle switches sign
  // for loop is used because while loop is not allowed
  // 30 is the recursion maximum which translates to roughly 30 turn in one
  // direction before breaking
  for (int i = 0; i < 30; i++) {
    angle -= initSign * 360;
    if (std::fabs(angle) <= 180) {
      break;
    }
  }

  return units::degree_t(std::lround(angle) % 360);
}

void PigeonGyro::Set(units::degree_t heading) {
  int err = pigeon->SetFusedHeading(heading.value(), 30);
  m_angle = heading.value();
  m_rate = 0;
  if (err != 0)
    std::printf("Set Position Error\n");
}
