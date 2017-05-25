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
#ifndef RANGE_SENSORHANDLER_HPP_
#define RANGE_SENSORHANDLER_HPP_

namespace msf_range_sensor {
RangeSensorHandler::RangeSensorHandler(
    msf_core::MSF_SensorManager<msf_updates::EKFState>& meas,
    std::string topic_namespace, std::string parameternamespace)
    : SensorHandler<msf_updates::EKFState>(meas, topic_namespace,
                                           parameternamespace),
      delay_(0),
      n_zp_(1e-6) {
  ros::NodeHandle pnh("~/range_sensor");
  ros::NodeHandle nh("msf_updates");

  pnh.param("enable_mah_outlier_rejection", enable_mah_outlier_rejection_, false);
  pnh.param("mah_threshold", mah_threshold_, msf_core::kDefaultMahThreshold_);

  subRange_ =
      nh.subscribe<sensor_msgs::Range>
      ("range_height", 20, &RangeSensorHandler::MeasurementCallback, this);

  memset(heightbuff, 0, sizeof(double) * heightbuffsize);

}

void RangeSensorHandler::SetNoises(double n_zp) {
  n_zp_ = n_zp;
}

void RangeSensorHandler::SetDelay(double delay) {
  delay_ = delay;
}

void RangeSensorHandler::MeasurementCallback(
    const sensor_msgs::RangeConstPtr & msg) {

  received_first_measurement_ = true;

  this->SequenceWatchDog(msg->header.seq, subRange_.getTopic());
  MSF_INFO_STREAM_ONCE(
      "*** range sensor got first measurement from topic "
          << this->topic_namespace_ << "/" << subRange_.getTopic()
          << " ***");

  shared_ptr<range_measurement::RangeMeasurement> meas(
      new range_measurement::RangeMeasurement(
          n_zp_, true, this->sensorID, enable_mah_outlier_rejection_,
          mah_threshold_));
  meas->MakeFromSensorReading(msg, msg->header.stamp.toSec() - delay_);

  z_p_ = meas->z_p_;  // Store this for the init procedure.

  // Make averaged measurement.
  memcpy(heightbuff, heightbuff + 1, sizeof(double) * (heightbuffsize - 1));
  heightbuff[heightbuffsize - 1] = meas->z_p_(0);
  double sum = 0;
  for (int k = 0; k < heightbuffsize; ++k)
    sum += heightbuff[k];
  z_average_p(0) = sum / heightbuffsize;

  this->manager_.msf_core_->AddMeasurement(meas);
}
}  // namespace msf_range_sensor
#endif  // RANGE_SENSORHANDLER_HPP_
