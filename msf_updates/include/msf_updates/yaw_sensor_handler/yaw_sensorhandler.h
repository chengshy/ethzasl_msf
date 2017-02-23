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
#ifndef YAW_SENSOR_H
#define YAW_SENSOR_H

#include <queue>

#include <geometry_msgs/QuaternionStamped.h>
#include <msf_core/msf_sensormanagerROS.h>

#include <msf_updates/yaw_sensor_handler/yaw_measurement.h>

namespace msf_yaw_sensor {
class YawSensorHandler : public msf_core::SensorHandler<
    typename msf_updates::EKFState> {
 private:
  Eigen::Quaterniond z_q_;  ///< Quaternion measurement.
  double n_zq_;  ///< yaw measurement noise.
  double delay_;
  ros::Subscriber subYaw_;
  void MeasurementCallback(const geometry_msgs::QuaternionStampedConstPtr & msg);
 public:
  YawSensorHandler(
      msf_core::MSF_SensorManager<msf_updates::EKFState>& meas,
      std::string topic_namespace, std::string parameternamespace);
  // Used for the init.
  Eigen::Quaterniond GetYawMeasurement() {
    return z_q_;
  }
  void SetNoises(double n_zq);
  void SetDelay(double delay);
};
}  // namespace msf_yaw_sensor
#include "implementation/yaw_sensorhandler.hpp"
#endif  // YAW_SENSOR_H
