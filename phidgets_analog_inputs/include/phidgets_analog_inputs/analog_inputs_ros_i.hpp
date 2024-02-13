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

#ifndef PHIDGETS_ANALOG_INPUTS__ANALOG_INPUTS_ROS_I_HPP_
#define PHIDGETS_ANALOG_INPUTS__ANALOG_INPUTS_ROS_I_HPP_

#include <memory>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include "phidgets_api/analog_inputs.hpp"

namespace phidgets
{
/**
 * This class provides methods for retrieving and publishing voltage data.
 */
class AnalogInputsRosI final : public rclcpp::Node
{
public:
  /// @brief Constructor for the TemperatureRosI class
  explicit AnalogInputsRosI(const rclcpp::NodeOptions & options);

private:
  // Sensor handle
  std::unique_ptr<AnalogInputs> ais_;

  // Mutex for analog input
  std::mutex ai_mutex_;

  // Container to store latest sensor value
  double last_sensor_reading_{0.0};

  // A flag to indicate start of data streaming
  bool got_first_data_;

  // Publisher for voltage
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr voltage_pub_;

  // Timer callback function
  void timerCallback();

  // Shared timer object
  rclcpp::TimerBase::SharedPtr timer_;

  // rate at which sensor value is published
  double publish_rate_;

  /**
   * @brief Publish latest sensor value.
   * @return Void.
   */
  void publishLatest();

  /**
   * @brief Handle temperature callback.
   * @param sensor_value Raw data coming from sensor.
   * @return Void.
   */
  void sensorChangeCallback(double sensor_value);
};

}  // namespace phidgets

#endif  // PHIDGETS_ANALOG_INPUTS__ANALOG_INPUTS_ROS_I_HPP_
