// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "utils/regressions/SquareRoot.h"

#include <cmath>

using namespace hb;

SquareRoot::SquareRoot(double a, double b, double h, double k)
    : m_a(a), m_b(b), m_h(h), m_k(k) {}

double SquareRoot::Calculate(double x) const {
  return m_a * std::sqrt(m_b * x - m_h) + m_k;
}
