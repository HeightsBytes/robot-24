// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "utils/regressions/Logarithmic.h"

#include <cmath>

using namespace hb;

Logarithmic::Logarithmic(double a, double b) : m_a(a), m_b(b) {}

double Logarithmic::Calculate(double x) const {
  return m_a + m_b * std::log(x);  // std::log is natural log
}
