// Copyright 2025 ros2_control development team
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SWERVE_DRIVE_CONTROLLER__SWERVE_DRIVE_KINEMATICS_HPP_
#define SWERVE_DRIVE_CONTROLLER__SWERVE_DRIVE_KINEMATICS_HPP_

#include <angles/angles.h>
#include <array>
#include <cmath>
#include <iostream>
#include <memory>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

namespace swerve_drive_controller
{

/**
 * @brief Struct to represent the PCV kinematics command for a single caster.
 *
 * Both velocities are at the joint output shaft (after the gearbox).
 * The hardware interface applies gear ratios and the steer-drive coupling term
 * before sending to the motors.
 */
struct CasterCommand
{
  double steer_vel;  // Steer column angular velocity (rad/s, output shaft)
  double drive_vel;  // Drive wheel angular velocity (rad/s)
};

/**
 * @brief Struct to represent the odometry state of the robot.
 */
struct OdometryState
{
  double x;      // X position in meters
  double y;      // Y position in meters
  double theta;  // Orientation (yaw) in radians
  double vx;     // Linear velocity in the x direction (m/s)
  double vy;     // Linear velocity in the y direction (m/s)
  double wz;     // Angular velocity about the z-axis (rad/s)
};

class SwerveDriveKinematics
{
public:
  /// @brief Default Constructor
  SwerveDriveKinematics();

  /**
   * @brief Sets necessary params required for kinematics calculation.
   * @param wheel_base Distance between front and rear axle centerlines (meters).
   * @param track_width Distance between left and right caster hub centerlines (meters).
   * @param x_offset Optional global x offset (center-of-gravity correction).
   * @param y_offset Optional global y offset.
   * @attention Order enforced as: front_left, front_right, rear_left, rear_right.
   *            wheel_positions_[i] stores the hub position (h_x, h_y) of caster i.
   */
  void calculate_wheel_position(
    double wheel_base, double track_width, double x_offset = 0.0, double y_offset = 0.0);

  /**
   * @brief Compute per-caster steer and drive velocity commands using PCV C-matrix kinematics.
   *
   * Implements the powered-caster vehicle kinematic model from base_controller.py (tidybot2).
   * The C matrix maps body-frame velocity [vx, vy, wz] to joint-space velocities
   * [steer_vel_i, drive_vel_i] for each caster, given the current steer angles.
   *
   * @param vx        Desired body-frame x velocity (m/s).
   * @param vy        Desired body-frame y velocity (m/s).
   * @param wz        Desired yaw rate (rad/s).
   * @param steer_angles Current steer column angles for each caster (rad), read from state.
   * @param wheel_radius Wheel radius (m).
   * @param b_x       Caster arm x-offset from pivot to wheel contact (m). Must be non-zero.
   * @param b_y       Caster arm lateral y-offset (m).
   * @return Array of CasterCommand (steer_vel rad/s, drive_vel rad/s) for each caster.
   */
  std::array<CasterCommand, 4> compute_caster_commands(
    double vx, double vy, double wz,
    const std::array<double, 4> & steer_angles,
    double wheel_radius, double b_x, double b_y);

  /**
   * @brief Update the odometry based on wheel velocities and elapsed time.
   * @param wheel_velocities Array of measured drive wheel linear velocities (m/s).
   * @param steering_angles Array of measured steer column angles (radians).
   * @param dt Time step (seconds).
   * @return Updated odometry state.
   */
  OdometryState update_odometry(
    const std::array<double, 4> & wheel_velocities_, const std::array<double, 4> & steering_angles_,
    double dt);

private:
  // Hub positions for each caster: (h_x, h_y) relative to robot centre.
  // Order: front_left, front_right, rear_left, rear_right.
  std::array<std::pair<double, double>, 4> wheel_positions_;
  OdometryState odometry_;  // Current odometry of the robot
};
}  // namespace swerve_drive_controller

#endif  // SWERVE_DRIVE_CONTROLLER__SWERVE_DRIVE_KINEMATICS_HPP_
