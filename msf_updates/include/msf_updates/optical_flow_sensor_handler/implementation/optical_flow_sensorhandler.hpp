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
#include <msf_core/eigen_utils.h>
#ifndef OPTICAL_FLOW_SENSORHANDLER_HPP_
#define OPTICAL_FLOW_SENSORHANDLER_HPP_

namespace msf_optical_flow_sensor {
OptiFlowSensorHandler::OptiFlowSensorHandler(
    msf_core::MSF_SensorManager<msf_updates::EKFState>& meas,
    std::string topic_namespace, std::string parameternamespace)
    : SensorHandler<msf_updates::EKFState>(meas, topic_namespace,
                                           parameternamespace), delay_(0),
      n_zv_(1e-6) {
  ros::NodeHandle pnh("~/optiflow");
  ros::NodeHandle nh("msf_updates");

  pnh.param("enable_mah_outlier_rejection", enable_mah_outlier_rejection_, false);
  pnh.param("mah_threshold", mah_threshold_, msf_core::kDefaultMahThreshold_);

  subOptiFlow_ =
      nh.subscribe<geometry_msgs::TwistStamped>
      ("optiflow", 20, &OptiFlowSensorHandler::MeasurementCallback, this);
}

void OptiFlowSensorHandler::SetNoises(double n_zv) {
  n_zv_ = n_zv;
}

void OptiFlowSensorHandler::SetDelay(double delay) {
  delay_ = delay;
}

void OptiFlowSensorHandler::MeasurementCallback(
    const geometry_msgs::TwistStampedConstPtr& msg) {

  received_first_measurement_ = true;

  this->SequenceWatchDog(msg->header.seq, subOptiFlow_.getTopic());
  MSF_INFO_STREAM_ONCE(
      "*** opti flow sensor got first measurement from topic "
          << this->topic_namespace_ << "/" << subOptiFlow_.getTopic()
          << " ***");

  shared_ptr<optical_flow_measurement::OptiFlowMeasurement> meas(
      new optical_flow_measurement::OptiFlowMeasurement(
          n_zv_, true, this->sensorID, enable_mah_outlier_rejection_,
          mah_threshold_));
  meas->MakeFromSensorReading(msg, msg->header.stamp.toSec() - delay_);

  z_v_ = meas->z_v_;  // Store this for the init procedure.

  this->manager_.msf_core_->AddMeasurement(meas);
}
}  // namespace msf_optical_flow_sensor
#endif  // OPTICAL_FLOW_SENSORHANDLER_HPP_
