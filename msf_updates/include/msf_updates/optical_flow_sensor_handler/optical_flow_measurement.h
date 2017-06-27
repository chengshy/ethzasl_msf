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
#ifndef OPTICAL_FLOW_MEASUREMENT_HPP_
#define OPTICAL_FLOW_MEASUREMENT_HPP_

#include <msf_core/msf_measurement.h>
#include <msf_core/msf_core.h>
#include <geometry_msgs/TwistStamped.h>

namespace optical_flow_measurement {
enum {
  nMeasurements = 2
};
/**
 * \brief A measurement as provided by a optical flow sensor.
 */
typedef msf_core::MSF_Measurement<geometry_msgs::TwistStamped,
    Eigen::Matrix<double, nMeasurements, nMeasurements>, msf_updates::EKFState> OptiFlowMeasurementBase;
struct OptiFlowMeasurement : public OptiFlowMeasurementBase {
 private:
  typedef OptiFlowMeasurementBase Measurement_t;
  typedef Measurement_t::Measurement_ptr measptr_t;

  virtual void MakeFromSensorReadingImpl(measptr_t msg) {
    Eigen::Matrix<double, nMeasurements,
        msf_core::MSF_Core<msf_updates::EKFState>::nErrorStatesAtCompileTime> H_old;
    Eigen::Matrix<double, nMeasurements, 1> r_old;

    H_old.setZero();

    // Get measurements.
    //
    z_v_ = Eigen::Matrix<double, 2, 1>::Constant(msg->twist.linear.x, msg->twist.linear.y);

    const double s_zv = n_zv_ * n_zv_;
    R_ = (Eigen::Matrix<double, nMeasurements, 1>() << s_zv, s_zv).finished()
        .asDiagonal();
  }
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Matrix<double, 2, 1> z_v_;  /// Range measurement.
  double n_zv_;  /// Range measurement noise.

  typedef msf_updates::EKFState EKFState_T;
  typedef EKFState_T::StateSequence_T StateSequence_T;
  typedef EKFState_T::StateDefinition_T StateDefinition_T;
  virtual ~OptiFlowMeasurement() {}
  OptiFlowMeasurement(double n_zv, bool isabsoluteMeasurement, int sensorID,
                      bool enable_mah_outlier_rejection, double mah_threshold)
      : OptiFlowMeasurementBase(isabsoluteMeasurement, sensorID,
                                enable_mah_outlier_rejection, mah_threshold),
        n_zv_(n_zv) {}
  virtual std::string Type() { return "opti_flow"; }

  virtual void CalculateH(
      shared_ptr<EKFState_T> state_in,
      Eigen::Matrix<double, nMeasurements, msf_core::MSF_Core<EKFState_T>::nErrorStatesAtCompileTime>& H)
  {
    const EKFState_T& state = *state_in;
    H.setZero();

    //Get rotation matrix
    //Eigen::Matrix3d C_iw = state.Get<StateDefinition_T::q>().conjugate().toRotationMatrix();

    //Eigen::Vector3d v_iw = state.Get<StateDefinition_T::v>();

    //Eigen::Matrix3d v_body_skew = Skew(C_iw * v_iw);

    enum{
      idx_startcoor_v_ = msf_tmp::GetStartIndexInCorrection<StateSequence_T, StateDefinition_T::v>::value,
      //idx_startcoor_q_ = msf_tmp::GetStartIndexInCorrection<StateSequence_T, StateDefinition_T::q>::value,
    };

    H.block<2,2>(0, idx_startcoor_v_) = Eigen::Matrix2d::Identity();
    //H.block<2,3>(0, idx_startcoor_q_) = v_body_skew.block<2,3>(0,0);
  }

  /**
   * The method called by the msf_core to apply the measurement represented by
   * this object.
   */
  virtual void Apply(shared_ptr<EKFState_T> non_const_state,
                     msf_core::MSF_Core<EKFState_T>& core) {
    // Init variables.
    const EKFState_T& state = *non_const_state;

    Eigen::Matrix<double, nMeasurements,
        msf_core::MSF_Core<EKFState_T>::nErrorStatesAtCompileTime> H_new;
    Eigen::Matrix<double, nMeasurements, 1> r_old;

    if (non_const_state->time == msf_core::constants::INVALID_TIME) {
      MSF_WARN_STREAM(
          "Apply optical flow update was called with an invalid state.");
      return;  // Early abort.
    }

    CalculateH(non_const_state, H_new);

    Eigen::Quaterniond q = state.Get<StateDefinition_T::q>();
    double yaw = std::atan2(2.*(q.w()*q.z() + q.x()*q.y()), 1.-2.*(q.y()*q.y() + q.z()*q.z()));
    Eigen::Vector2d v_w = state.Get<StateDefinition_T::v>().block<2,1>(0,0);
    Eigen::Vector2d v_m(
        std::cos(yaw) * z_v_(0) - std::sin(yaw) * z_v_(1),
        std::sin(yaw) * z_v_(0) + std::cos(yaw) * z_v_(1));

    r_old = v_m - v_w;

    this->CalculateAndApplyCorrection(non_const_state, core, H_new, r_old, R_);
  }
};
}  // namespace optical_flow_measurement
#endif  // OPTICAL_FLOW_MEASUREMENT_HPP_
