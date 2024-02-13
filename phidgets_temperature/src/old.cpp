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

#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/float64.hpp>

#include "phidgets_temperature/temperature_ros_i.hpp"

namespace phidgets {

TemperatureRosI::TemperatureRosI(const rclcpp::NodeOptions& options)
    : rclcpp::Node("phidgets_temperature_node", options)
{
    setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

    RCLCPP_INFO(get_logger(), "Starting Phidgets Temperature");

    int serial_num = 664919;

    //int data_interval_ms = 10;

    publish_rate_ = 10.0;

    std::lock_guard<std::mutex> lock(temperature_mutex_);
    std::string server = "server";
    std::string address = "192.168.1.1";
    int port = 5661;
    std::string password = "";
    int flags = 0;
    PhidgetNet_addServer(server.c_str(), address.c_str(), port, password.c_str(), flags);
    PhidgetTemperatureSensor_create(&temperature_handle_);
    Phidget_setIsRemote(reinterpret_cast<PhidgetHandle>(temperature_handle_), 1);
    Phidget_setDeviceSerialNumber(reinterpret_cast<PhidgetHandle>(temperature_handle_), serial_num);
    PhidgetTemperatureSensor_setOnTemperatureChangeHandler(
        temperature_handle_, TemperatureChangeHandler, this);
    Phidget_openWaitForAttachment(reinterpret_cast<PhidgetHandle>(temperature_handle_), 20000);
    temperature_pub_ =
        this->create_publisher<std_msgs::msg::Float64>("temperature", 1);
    std::function<void(double)> temperature_handler = std::bind(&TemperatureRosI::temperatureChangeCallback, this,
                      std::placeholders::_1);
    got_first_data_ = false;

    double pub_msec = 1000.0 / publish_rate_;
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int64_t>(pub_msec)),
        std::bind(&TemperatureRosI::timerCallback, this));

}

void TemperatureRosI::publishLatest()
{
    auto msg = std::make_unique<std_msgs::msg::Float64>();
    msg->data = last_temperature_reading_;
    temperature_pub_->publish(std::move(msg));
}

void TemperatureRosI::timerCallback()
{
    std::lock_guard<std::mutex> lock(temperature_mutex_);
    if (got_first_data_)
    {
        publishLatest();
    }
}

void TemperatureRosI::temperatureChangeHandler(double temperature) const
{
    temperature_handler_(temperature);
}

void Temperature::TemperatureChangeHandler(
    PhidgetTemperatureSensorHandle /* temperature_handle */, void *ctx,
    double temperature)
{
    (reinterpret_cast<Temperature *>(ctx))
        ->temperatureChangeHandler(temperature);
}


void TemperatureRosI::temperatureChangeCallback(double temperature)
{
    std::lock_guard<std::mutex> lock(temperature_mutex_);
    last_temperature_reading_ = temperature;

    if (!got_first_data_)
    {
        got_first_data_ = true;
    }

    if (publish_rate_ <= 0.0)
    {
        publishLatest();
    }
}

}  // namespace phidgets

RCLCPP_COMPONENTS_REGISTER_NODE(phidgets::TemperatureRosI)





