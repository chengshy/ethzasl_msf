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
#ifndef OPTIFLOW_YAW_RANGE_MEASUREMENTMANAGER_H
#define OPTIFLOW_YAW_RANGE_MEASUREMENTMANAGER_H

#include <ros/ros.h>

#include <msf_core/msf_core.h>
#include <msf_core/msf_sensormanagerROS.h>
#include <msf_core/msf_IMUHandler_ROS.h>
#include "msf_statedef.hpp"
#include <msf_updates/optical_flow_sensor_handler/optical_flow_sensorhandler.h>
#include <msf_updates/optical_flow_sensor_handler/optical_flow_measurement.h>
#include <msf_updates/range_sensor_handler/range_sensorhandler.h>
#include <msf_updates/range_sensor_handler/range_measurement.h>
#include <msf_updates/yaw_sensor_handler/yaw_measurement.h>
#include <msf_updates/yaw_sensor_handler/yaw_sensorhandler.h>
#include <msf_updates/OptiFlowYawRangeSensorConfig.h>

namespace msf_optiflow_yaw_range_sensor {

typedef msf_updates::OptiFlowYawRangeSensorConfig Config_T;
typedef dynamic_reconfigure::Server<Config_T> ReconfigureServer;
typedef shared_ptr<ReconfigureServer> ReconfigureServerPtr;

class OptiFlowYawRangeSensorManager : public msf_core::MSF_SensorManagerROS<
    msf_updates::EKFState> {
  friend class msf_optical_flow_sensor::OptiFlowSensorHandler;
  friend class msf_yaw_sensor::YawSensorHandler;
  friend class msf_range_sensor::RangeSensorHandler;
 public:
  typedef msf_updates::EKFState EKFState_T;
  typedef EKFState_T::StateSequence_T StateSequence_T;
  typedef EKFState_T::StateDefinition_T StateDefinition_T;

  OptiFlowYawRangeSensorManager(
      ros::NodeHandle pnh = ros::NodeHandle("~/optiflow_yaw_range_sensor")) {
    imu_handler_.reset(
        new msf_core::IMUHandler_ROS<msf_updates::EKFState>(*this, "msf_core",
                                                            "imu_handler"));

    optiflow_handler_.reset(
        new msf_optical_flow_sensor::OptiFlowSensorHandler(*this, "", "optiflow_sensor"));
    AddHandler(optiflow_handler_);

    range_handler_.reset(
        new msf_range_sensor::RangeSensorHandler(*this, "", "range_sensor"));
    AddHandler(range_handler_);

    yaw_handler_.reset(
        new msf_yaw_sensor::YawSensorHandler(*this, "", "yaw_sensor"));
    AddHandler(yaw_handler_);

    reconf_server_.reset(new ReconfigureServer(pnh));
    ReconfigureServer::CallbackType f = boost::bind(
        &OptiFlowYawRangeSensorManager::Config, this, _1, _2);
    reconf_server_->setCallback(f);
  }
  virtual ~OptiFlowYawRangeSensorManager() {
  }

  virtual const Config_T& Getcfg() {
    return config_;
  }

 private:
  shared_ptr<msf_core::IMUHandler_ROS<msf_updates::EKFState> > imu_handler_;
  shared_ptr<msf_optical_flow_sensor::OptiFlowSensorHandler> optiflow_handler_;
  shared_ptr<msf_range_sensor::RangeSensorHandler> range_handler_;
  shared_ptr<msf_yaw_sensor::YawSensorHandler> yaw_handler_;

  Config_T config_;
  ReconfigureServerPtr reconf_server_;

  /**
   * \brief Dynamic reconfigure callback.
   */
  virtual void Config(Config_T &config, uint32_t level) {
    config_ = config;
    optiflow_handler_->SetNoises(config.optiflow_noise_meas);
    optiflow_handler_->SetDelay(config.optiflow_delay);
    range_handler_->SetNoises(config.range_noise_meas_p);
    range_handler_->SetDelay(config.range_delay);
    yaw_handler_->SetNoises(config.yaw_noise_meas);
    yaw_handler_->SetDelay(config.yaw_delay);

    if ((level & msf_updates::OptiFlowYawRangeSensor_INIT_FILTER)
        && config.core_init_filter == true) {
      Init(1.0);
      config.core_init_filter = false;
    }
  }

  void Init(double scale) const {
    if (scale < 0.001) {
      MSF_WARN_STREAM("init scale is "<<scale<<" correcting to 1");
      scale = 1;
    }

    Eigen::Matrix<double, 3, 1> p, v, b_w, b_a, g, w_m, a_m;
    Eigen::Quaternion<double> q;
    msf_core::MSF_Core<EKFState_T>::ErrorStateCov P;
    //Eigen::Matrix<double, 1, 1> b_yaw;
    //Eigen::Matrix<double, 1, 1> b_p;

    // Init values.
    g << 0, 0, 9.81;  /// Gravity.
    b_w << 0, 0, 0;		/// Bias gyroscopes.
    b_a << 0, 0, 0;		/// Bias accelerometer.

    v << 0, 0, 0;			/// Robot velocity (IMU centered).
    w_m << 0, 0, 0;		/// Initial angular velocity.

    //b_yaw << 0;
    //b_p << 0;

    q = Eigen::Quaterniond::Identity();

    P.setZero();  // Error state covariance; if zero, a default initialization
                  // in msf_core is used

    p.block<1,1>(2,0) = range_handler_->GetRangeMeasurement();

    MSF_INFO_STREAM(
        "initial measurement pos:[" << p.transpose() << "] orientation: " << STREAMQUAT(q));

    // check if we have already input from the measurement sensor
    if (optiflow_handler_->ReceivedFirstMeasurement())
      MSF_WARN_STREAM(
          "No measurements received yet from optical flow");

    if (range_handler_->ReceivedFirstMeasurement())
      MSF_WARN_STREAM(
          "No measurements received yet to initialize position z - using 0");

    if (yaw_handler_ -> ReceivedFirstMeasurement())
      MSF_WARN_STREAM(
          "No measurements received yet to initialize yaw - using 0");

    ros::NodeHandle pnh("~");
    a_m = q.inverse() * g;			    /// Initial acceleration.

    //prepare init "measurement"
    // True means that we will also set the initialsensor readings.
    shared_ptr < msf_core::MSF_InitMeasurement<EKFState_T>
        > meas(new msf_core::MSF_InitMeasurement<EKFState_T>(true));

    meas->SetStateInitValue < StateDefinition_T::p > (p);
    meas->SetStateInitValue < StateDefinition_T::v > (v);
    meas->SetStateInitValue < StateDefinition_T::q > (q);
    meas->SetStateInitValue < StateDefinition_T::b_w > (b_w);
    meas->SetStateInitValue < StateDefinition_T::b_a > (b_a);
    //meas->SetStateInitValue < StateDefinition_T::b_yaw> (b_yaw);
    //meas->SetStateInitValue < StateDefinition_T::b_yaw> (b_p);

    SetStateCovariance(meas->GetStateCovariance());  // Call my set P function.
    meas->Getw_m() = w_m;
    meas->Geta_m() = a_m;
    meas->time = ros::Time::now().toSec();

    // Call initialization in core.
    msf_core_->Init(meas);
  }

  // Prior to this call, all states are initialized to zero/identity.
  virtual void ResetState(EKFState_T& state) const {
    UNUSED(state);
  }
  virtual void InitState(EKFState_T& state) const {
    UNUSED(state);
  }

  virtual void CalculateQAuxiliaryStates(EKFState_T& state, double dt) const {
    //const msf_core::Vector1 nb_yaw = msf_core::Vector1::Constant(
    //    config_.yaw_noise_bias);
    //const msf_core::Vector1 nb_p = msf_core::Vector1::Constant(
    //    config_.range_noise_bias_p);

    // Compute the blockwise Q values and store them with the states,
    //these then get copied by the core to the correct places in Qd.
    //state.GetQBlock<StateDefinition_T::b_yaw>() = 
    //    (dt * nb_yaw.cwiseProduct(nb_yaw)).asDiagonal();
    //state.GetQBlock<StateDefinition_T::b_p>() = 
    //    (dt * nb_p.cwiseProduct(nb_p)).asDiagonal();
  }

  virtual void SetStateCovariance(
      Eigen::Matrix<double, EKFState_T::nErrorStatesAtCompileTime,
          EKFState_T::nErrorStatesAtCompileTime>& P) const {
    UNUSED(P);
    // Nothing, we only use the simulated cov for the core plus diagonal for the
    // rest.
  }

  virtual void AugmentCorrectionVector(
      Eigen::Matrix<double, EKFState_T::nErrorStatesAtCompileTime, 1>& correction) const {
    UNUSED(correction);
  }

  virtual void SanityCheckCorrection(
      EKFState_T& delaystate,
      const EKFState_T& buffstate,
      Eigen::Matrix<double, EKFState_T::nErrorStatesAtCompileTime, 1>& correction) const {
    UNUSED(delaystate);
    UNUSED(buffstate);
    UNUSED(correction);
  }
};
}
#endif  // OPTIFLOW_YAW_RANGE_MEASUREMENTMANAGER_H
