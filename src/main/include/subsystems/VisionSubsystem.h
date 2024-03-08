// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc2/command/SubsystemBase.h>
#include <photonlib/PhotonCamera.h>
#include <photonlib/PhotonPoseEstimator.h>
#include <units/angle.h>

#include <optional>
#include <vector>

#include "utils/cams/PosePacket.h"

/******************************************************************************\
 *                                                                            *
 * To update the Orange Pi:                                                   *
 * 1. Download the latest photonvision .jar file                              *
 * 2. Go into bash (windows subsystem for linux required)                     *
 * 3. scp [photonvision jar].jar orangepi@[module name]:~/                    *
 * 4. ssh orangepi@[module name]                                              *
 * 5. sudo mv [photonvision jar].jar /opt/photonvision/photonvision.jar       *
 * 6. sudo systemctl restart photonvision.service                             *
 * 7. sudo reboot now                                                         *
 *                                                                            *
\******************************************************************************/
class VisionSubsystem : public frc2::SubsystemBase {
 public:
  void Periodic() override;

  //   static VisionSubsystem& GetInstance();

  std::vector<PosePacket> GetPose();

  std::optional<units::degree_t> AngleToStage() const;

  std::optional<frc::Pose3d> GetTagPose(int id) const;

  photonlib::PhotonPipelineResult GetLeftResult();
  photonlib::PhotonPipelineResult GetRightResult();

  static std::optional<photonlib::PhotonTrackedTarget> HasResult(
      std::span<const photonlib::PhotonTrackedTarget> result, int ID);

 private:
  VisionSubsystem();

  void UpdatePacket();

  std::optional<PosePacket> PhotonToPosePacket(
      std::optional<photonlib::EstimatedRobotPose> pose);

  frc::AprilTagFieldLayout m_layout =
      frc::LoadAprilTagLayoutField(frc::AprilTagField::k2024Crescendo);

  photonlib::PhotonPoseEstimator m_rightEst;
  photonlib::PhotonPoseEstimator m_leftEst;

  std::vector<PosePacket> m_packets;
};
