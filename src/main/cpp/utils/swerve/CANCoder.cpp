// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "utils/swerve/CANCoder.h"

using namespace hb;
using namespace ctre::phoenix::sensors;

S_CANCoder::S_CANCoder(int Id, double offset = 0)
    : CANCoder(Id), m_offset(offset), m_ID(Id) {
  std::printf("CANCoder: %i, reading %5.2f\n", Id, GetAbsolutePosition() - 180);
}

units::radian_t S_CANCoder::Get() {
  double rv = GetAbsolutePosition() - m_offset;

  if (rv < 0.0)
    rv += 360.0;

  if (rv > 360.0)
    rv -= 360.0;

  double a = (std::numbers::pi * 2 * (rv / 360.0)) - (std::numbers::pi);

  return units::radian_t(a);
}
