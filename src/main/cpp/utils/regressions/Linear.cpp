// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "utils/regressions/Linear.h"

using namespace hb;

Linear::Linear(double a, double b) : m_a(a), m_b(b) {}

double Linear::Calculate(double x) const {
  return m_a * x + m_b;
}
