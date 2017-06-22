/*
 * Copyright (C) 2012-2013 Simon Lynen, ASL, ETH Zurich, Switzerland
 * You can contact the author at <slynen at ethz dot ch>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef OPTICAL_FLOW_SENSOR_H
#define OPTICAL_FLOW_SENSOR_H

#include <queue>

#include <geometry_msgs/TwistStamped.h>
#include <msf_core/msf_sensormanagerROS.h>

#include <msf_updates/optical_flow_sensor_handler/optical_flow_measurement.h>

namespace msf_optical_flow_sensor {
class OptiFlowSensorHandler : public msf_core::SensorHandler<
    typename msf_updates::EKFState> {
 private:
  Eigen::Matrix<double, 2, 1> z_v_;  ///< Range measurement.
  double n_zv_;  ///< Range measurement noise.
  double delay_;
  ros::Subscriber subOptiFlow_;
  void MeasurementCallback(const geometry_msgs::TwistStampedConstPtr& msg);
 public:
  OptiFlowSensorHandler(
      msf_core::MSF_SensorManager<msf_updates::EKFState>& meas,
      std::string topic_namespace, std::string parameternamespace);
  // Used for the init.
  Eigen::Matrix<double, 2, 1> GetOptiFlowMeasurement() {
    return z_v_;
  }
  // Setters for configure values.
  void SetNoises(double n_zv);
  void SetDelay(double delay);
};
}  // namespace msf_optical_flow_sensor
#include "implementation/optical_flow_sensorhandler.hpp"
#endif  // OPTICAL_FLOW_SENSOR_H
