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
#ifndef VISION_XY_SENSORHANDLER_HPP_
#define VISION_XY_SENSORHANDLER_HPP_
#include <msf_core/msf_types.h>
#include <msf_core/eigen_utils.h>

namespace msf_vision_xy_sensor {
template<typename MANAGER_TYPE>
VisionXYSensorHandler<MANAGER_TYPE>::VisionXYSensorHandler(
    MANAGER_TYPE& meas, std::string topic_namespace,
    std::string parameternamespace)
    : SensorHandler<msf_updates::EKFState>(meas, topic_namespace,
                                           parameternamespace),
      n_zp_(1e-5),
      n_zq_(1e-5),
      delay_(0) {
  ros::NodeHandle pnh("~/vision_sensor");
  pnh.param("vision_absolute_measurements", provides_absolute_measurements_,
            false);
  pnh.param("enable_mah_outlier_rejection", enable_mah_outlier_rejection_, false);
  pnh.param("mah_threshold", mah_threshold_, msf_core::kDefaultMahThreshold_);


  MSF_INFO_STREAM_COND(provides_absolute_measurements_, "Vision sensor is "
                       "handling measurements as absolute values");
  MSF_INFO_STREAM_COND(!provides_absolute_measurements_, "Vision sensor is "
                       "handling measurements as relative values");

  ros::NodeHandle nh("msf_updates");

  subPoseStamped_ =
      nh.subscribe<geometry_msgs::PoseStamped>
  ("vision_input", 20, &VisionSensorHandler::MeasurementCallback, this);

  z_p_.setZero();
  z_q_.setIdentity();

}

template<typename MANAGER_TYPE>
void VisionXYSensorHandler<MANAGER_TYPE>::SetNoises(double n_zp, double n_zq) {
  n_zp_ = n_zp;
  n_zq_ = n_zq;
}

template<typename MANAGER_TYPE>
void VisionXYSensorHandler<MANAGER_TYPE>::SetDelay(double delay) {
  delay_ = delay;
}


template<typename MANAGER_TYPE>
void VisionXYSensorHandler<MANAGER_TYPE>::MeasurementCallback(
    const geometry_msgs::PoseStampedConstPtr & msg) {
  this->SequenceWatchDog(msg->header.seq, subPoseStamped_.getTopic());

  received_first_measurement_ = true;


  MSF_INFO_STREAM_ONCE(
      "*** vision sensor got first measurement from topic "
          << this->topic_namespace_ << "/" << subPoseStamped_.getTopic()
          << " ***");

  // Get the fixed states.
  int fixedstates = 0;
  static_assert(msf_updates::EKFState::nStateVarsAtCompileTime < 32, "Your state "
      "has more than 32 variables. The code needs to be changed here to have a "
      "larger variable to mark the fixed_states");

  // Get all the fixed states and set flag bits.
  MANAGER_TYPE* mngr = dynamic_cast<MANAGER_TYPE*>(&manager_);

  if (mngr) {
    if (mngr->Getcfg().vision_fixed_p_ip) {
      fixedstates |= 1 << msf_updates::EKFState::StateDefinition_T::p_ip;
    }
  }

  shared_ptr<msf_updates::vision_xy_measurement::VisionXYMeasurement> meas(
      new msf_updates::vision_xy_measurement::VisionXYMeasurement(
      n_zp_, n_zq_, provides_absolute_measurements_,
      this->sensorID, fixedstates, enable_mah_outlier_rejection_,
      mah_threshold_));

  meas->MakeFromSensorReading(msg, msg->header.stamp.toSec() - delay_);

  z_p_ = meas->z_p_;  // Store this for the init procedure.
  z_q_ = meas->z_q_;  // Store this for the init procedure.

  this->manager_.msf_core_->AddMeasurement(meas);

}

}  // namespace msf_vision_xy_sensor
#endif  // VISION_XY_SENSORHANDLER_HPP_
