// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "utils/regressions/Exponential.h"

#include <cmath>

using namespace hb;

Exponential::Exponential(double a, double b) : m_a(a), m_b(b) {}

double Exponential::Calculate(double x) const {
  return m_a * std::pow(m_b, x);
}
