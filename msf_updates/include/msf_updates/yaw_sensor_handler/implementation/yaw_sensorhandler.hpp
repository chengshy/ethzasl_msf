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
#ifndef YAW_SENSORHANDLER_HPP_
#define YAW_SENSORHANDLER_HPP_

namespace msf_yaw_sensor {
YawSensorHandler::YawSensorHandler(
    msf_core::MSF_SensorManager<msf_updates::EKFState>& meas,
    std::string topic_namespace, std::string parameternamespace)
    : SensorHandler<msf_updates::EKFState>(meas, topic_namespace,
                                           parameternamespace),
      n_zq_(1e-5),
      delay_(0){
  ros::NodeHandle pnh("~/yaw_sensor");
  ros::NodeHandle nh("msf_updates");

  pnh.param("enable_mah_outlier_rejection", enable_mah_outlier_rejection_, false);
  pnh.param("mah_threshold", mah_threshold_, msf_core::kDefaultMahThreshold_);

  z_q_.setIdentity();

  subYaw_ =
      nh.subscribe<geometry_msgs::QuaternionStamped>
      ("yaw_input", 20, &YawSensorHandler::MeasurementCallback, this);
}

void YawSensorHandler::SetNoises(double n_zq) {
  n_zq_ = n_zq;
}

void YawSensorHandler::MeasurementCallback(
    const geometry_msgs::QuaternionStampedConstPtr & msg) {

  received_first_measurement_ = true;

  this->SequenceWatchDog(msg->header.seq, subYaw_.getTopic());
  MSF_INFO_STREAM_ONCE(
      "*** Yaw sensor got first measurement from topic "
          << this->topic_namespace_ << "/" << subYaw_.getTopic()
          << " ***");

  shared_ptr<msf_updates::yaw_measurement::YawMeasurement> meas(
      new msf_updates::yaw_measurement::YawMeasurement(
          n_zq_, true, this->sensorID, enable_mah_outlier_rejection_,
          mah_threshold_));
  meas->MakeFromSensorReading(msg, msg->header.stamp.toSec() - delay_);

  z_q_ = meas->z_q_;  // Store this for the init procedure.

  this->manager_.msf_core_->AddMeasurement(meas);
}

void YawSensorHandler::SetDelay(double delay){
  delay_ = delay;
}
}  // namespace msf_yaw_sensor
#endif  // YAW_SENSORHANDLER_HPP_
