/*
 * Copyright (C) 2012-2013 Simon Lynen, ASL, ETH Zurich, Switzerland
 * You can contact the author at <slynen at ethz dot ch>
 * Copyright (C) 2011-2012 Stephan Weiss, ASL, ETH Zurich, Switzerland
 * You can contact the author at <stephan dot weiss at ieee dot org>
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
#ifndef VISION_XY_SENSOR_H_
#define VISION_XY_SENSOR_H_

#include <msf_core/msf_sensormanagerROS.h>
#include <geometry_msgs/PoseStamped.h>
#include <msf_updates/vision_xy_sensor_handler/vision_xy_measurement.h>

namespace msf_vision_xy_sensor {

template<typename MANAGER_TYPE>
class VisionXYSensorHandler : public msf_core::SensorHandler<
    typename msf_updates::EKFState> {
 private:

  Eigen::Matrix<double, 2, 1> z_p_;  ///< Position xy measurement.
  Eigen::Quaterniond z_q_;  ///< Quaternion measurement.
  double n_zp_;  ///< Position measurement noise.
  double n_zq_;  ///< Yaw measurement noise.
  double delay_;       ///< Delay to be subtracted from the ros-timestamp of
                       //the measurement provided by this sensor.

  ros::Subscriber subPoseStamped_;

  bool use_fixed_covariance_;  ///< Use fixed covariance set by dynamic reconfigure.
  bool provides_absolute_measurements_;  ///< Does this sensor measure relative or absolute values.

  void MeasurementCallback(const geometry_msgs::PoseStampedConstPtr& msg);

 public:
  VisionXYSensorHandler(MANAGER_TYPE& meas, std::string topic_namespace,
                        std::string parameternamespace);
  // Used for the init.
  Eigen::Matrix<double, 2, 1> GetPositionMeasurement() {
    return z_p_;
  }

  Eigen::Quaterniond GetYawMeasurement() {
    return z_q_;
  }
  // Setters for configure values.
  void SetNoises(double n_zp, double n_zq);
  void SetDelay(double delay);
};
}  // namespace msf_vision_xy_sensor

#include "implementation/vision_xy_sensorhandler.hpp"

#endif  // VISION_XY_SENSOR_H_
