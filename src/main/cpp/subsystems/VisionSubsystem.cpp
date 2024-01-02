// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/VisionSubsystem.h"

#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants.h"
#include "utils/cams/Limelight.h"
#include "utils/cams/PosePacket.h"

VisionSubsystem::VisionSubsystem()
    : m_rightEst(
          m_layout, photonlib::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR,
          std::move(photonlib::PhotonCamera("Arducam_OV9281_USB_Camera_Right")),
          VisionConstants::RightTransform),
      m_leftEst(
          m_layout, photonlib::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR,
          std::move(photonlib::PhotonCamera("Arducam_OV9281_USB_Camera_Left")),
          VisionConstants::LeftTransform) {
  m_leftEst.SetMultiTagFallbackStrategy(
      photonlib::PoseStrategy::LOWEST_AMBIGUITY);
  m_rightEst.SetMultiTagFallbackStrategy(
      photonlib::PoseStrategy::LOWEST_AMBIGUITY);
}

// This method will be called once per scheduler run
void VisionSubsystem::Periodic() {
  std::vector<PosePacket> packets;

  std::optional<PosePacket> llPose = hb::LimeLight::GetPose();
  if (llPose) {
    packets.emplace_back(llPose.value());
  }

  std::optional<photonlib::EstimatedRobotPose> estl = m_leftEst.Update();
  if (estl) {
    if (estl->strategy ==
        photonlib::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR) {
      packets.emplace_back(PhotonToPosePacket(estl).value());
    }
  }

  std::optional<photonlib::EstimatedRobotPose> estr = m_rightEst.Update();
  if (estr) {
    if (estr->strategy ==
        photonlib::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR) {
      packets.emplace_back(PhotonToPosePacket(estr).value());
    }
  }

  m_packets = packets;
}

VisionSubsystem& VisionSubsystem::GetInstance() {
  static VisionSubsystem inst;
  return inst;
}

std::vector<PosePacket> VisionSubsystem::GetPose() {
  return m_packets;
}

std::optional<PosePacket> VisionSubsystem::PhotonToPosePacket(
    std::optional<photonlib::EstimatedRobotPose> pose) {
  if (!pose.has_value())
    return std::nullopt;

  return PosePacket(pose.value().estimatedPose.ToPose2d(),
                    pose.value().timestamp);
}
