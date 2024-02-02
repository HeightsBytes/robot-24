// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <units/time.h>

#include <optional>

#include "PosePacket.h"

namespace hb {

  /**
   * @brief singleton util class for the one limelight on the robot
   */
  class LimeLight {
   public:
    /**
     * @brief Constructor is deleted as class will be static only
     */
    LimeLight() = delete;

    /**
     * @brief Enum for all LED modes
     *
     */
    enum class LEDMode { kPipeline = 0, kOff = 1, kOn = 2 };

    /**
     * @brief Enum for Camera Modes
     *
     */
    enum class CamMode { kProcessed = 0, kUnprocessed = 1 };

    /**
     * @brief enum for easier naming for pipelines
     */
    enum class Pipeline { kRetroReflective = 0, kAprilTag = 1 };

    /**
     * @returns true if a target is spotted
     */
    static bool HasTarget();

    /**
     * @returns The X offset to target
     */
    static double GetX();

    /**
     * @returns The Y offset to target
     */
    static double GetY();

    /**
     * @returns The area the target covers, meant for apriltags
     */
    static double GetA();

    /**
     * @param LEDMode enum class sets the LED on, off, or to the pipeline
     */
    static void SetLED(LEDMode);

    /**
     * @param CAMMode enum class
     *
     * @brief sets the Camera from processed to unprocessed
     */
    static void SetMode(CamMode);

    /**
     * @param Pipeline enum class
     * sets the camera to a specific pipeline ie Apriltag or retro reflective
     * tape
     */
    static void SetPipeline(Pipeline);

    /**
     * @brief Get the current Pipeline
     *
     * @return Pipeline
     */
    static Pipeline GetPipeline();

    /**
     * @brief Get the current Mode
     *
     * @return CamMode
     */
    static CamMode GetMode();

    /**
     * @brief Get the current LED state
     *
     * @return LEDMode
     */
    static LEDMode GetLED();

    /**
     * @brief Get the pose of the robot (If it exists). If one option is null,
     * they both are
     *
     * @return optional PosePacket{contains pose and timestamp}
     *
     * @warning you will need to handle nullopt
     */
    static std::optional<PosePacket> GetPose();
  };
}  // namespace hb
