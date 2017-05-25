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
#ifndef RANGE_SENSOR_H
#define RANGE_SENSOR_H

#include <queue>

#include <sensor_msgs/Range.h>
#include <msf_core/msf_sensormanagerROS.h>

#include <msf_updates/range_sensor_handler/range_measurement.h>

namespace msf_range_sensor {
class RangeSensorHandler : public msf_core::SensorHandler<
    typename msf_updates::EKFState> {
 private:
  enum {
    heightbuffsize = 10
  };
  Eigen::Matrix<double, 1, 1> z_p_;  ///< Range measurement.
  double n_zp_;  ///< Range measurement noise.
  double delay_;
  Eigen::Matrix<double, 1, 1> z_average_p;  ///<Averaged range measurement.
  double heightbuff[heightbuffsize];
  ros::Subscriber subRange_;
  void MeasurementCallback(const sensor_msgs::RangeConstPtr & msg);
 public:
  RangeSensorHandler(
      msf_core::MSF_SensorManager<msf_updates::EKFState>& meas,
      std::string topic_namespace, std::string parameternamespace);
  // Used for the init.
  Eigen::Matrix<double, 1, 1> GetRangeMeasurement() {
    return z_p_;
  }
  Eigen::Matrix<double, 1, 1> GetAveragedRangeMeasurement() {
    return z_average_p;
  }
  // Setters for configure values.
  void SetNoises(double n_zp);
  void SetDelay(double delay);
};
}  // namespace msf_range_sensor
#include "implementation/range_sensorhandler.hpp"
#endif  // RANGE_SENSOR_H
