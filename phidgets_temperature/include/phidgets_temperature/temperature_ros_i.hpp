/*
 * Copyright (c) 2019, Open Source Robotics Foundation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef PHIDGETS_TEMPERATURE__TEMPERATURE_ROS_I_HPP_
#define PHIDGETS_TEMPERATURE__TEMPERATURE_ROS_I_HPP_

#include <memory>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include "phidgets_api/temperature.hpp"

namespace phidgets
{
/**
 * This class provides methods for retrieving and publishing temperature data.
 */
class TemperatureRosI final : public rclcpp::Node
{
public:
  /// @brief Constructor for the TemperatureRosI class
  explicit TemperatureRosI(const rclcpp::NodeOptions & options);

private:
  // temperature sensor handle
  std::unique_ptr<Temperature> temperature_;

  // Mutex for temperature data
  std::mutex temperature_mutex_;

  // Store latest reading
  double last_temperature_reading_{0.0};

  // Flag to indicate start of the data streaming
  bool got_first_data_;

  // Temperature data publisher
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr temperature_pub_;

  // Timer callback for desired publishing frequency
  void timer_callback();

  // Shared timer object
  rclcpp::TimerBase::SharedPtr timer_;

  // Desired publishing frequency
  double publish_rate_;
  /**
   * @brief Publish latest temperature data.
   * @return Void.
   */
  void publish_latest();

  /**
   * @brief Handle temperature callback.
   * @param temperature Raw data coming from sensor.
   * @return Void.
   */
  void sensor_change_callback(double temperature);
};

}  // namespace phidgets

#endif  // PHIDGETS_TEMPERATURE__TEMPERATURE_ROS_I_HPP_
