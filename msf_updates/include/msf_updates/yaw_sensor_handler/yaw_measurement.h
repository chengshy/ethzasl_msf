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
#ifndef YAW_MEASUREMENT_HPP_
#define YAW_MEASUREMENT_HPP_

#include <msf_core/msf_types.h>
#include <msf_core/msf_measurement.h>
#include <msf_core/msf_core.h>

namespace msf_updates {
namespace yaw_measurement {
enum {
  nMeasurements = 1
};
/**
 * \brief A measurement as provided by a yaw tracking algorithm.
 * The measurement type is quaternion stamped. Convert to yaw later
 */
typedef msf_core::MSF_Measurement<geometry_msgs::QuaternionStamped,
    Eigen::Matrix<double, nMeasurements, nMeasurements>, msf_updates::EKFState> YawMeasurementBase;

struct YawMeasurement : public YawMeasurementBase {
 private:
  typedef YawMeasurementBase Measurement_t;
  typedef Measurement_t::Measurement_ptr measptr_t;

  virtual void MakeFromSensorReadingImpl(measptr_t msg) {
    Eigen::Matrix<double, nMeasurements,
        msf_core::MSF_Core<msf_updates::EKFState>::nErrorStatesAtCompileTime> H_old;
    Eigen::Matrix<double, nMeasurements, 1> r_old;

    H_old.setZero();

    // Get measurements.
    z_q_ = Eigen::Quaternion<double>(msg->quaternion.w,
                                     msg->quaternion.x,
                                     msg->quaternion.y,
                                     msg->quaternion.z);

    const double s_zq = n_zq_ * n_zq_;

    R_ = (Eigen::Matrix<double, nMeasurements, 1>() << s_zq).finished();
  }

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Quaternion<double> z_q_;  /// Attitude measurement camera seen from world.
  double n_zq_;  /// attitude measurement noise.

  typedef msf_updates::EKFState EKFState_T;
  typedef EKFState_T::StateDefinition_T StateDefinition_T;

  virtual ~YawMeasurement() {}
  YawMeasurement(double n_zq, int sensorID, bool isabsoluteMeasurement,
                 bool enable_mah_outlier_rejection, double mah_threshold)
      : YawMeasurementBase(isabsoluteMeasurement, sensorID,
                            enable_mah_outlier_rejection, mah_threshold),
        n_zq_(n_zq) {}
  virtual std::string Type() {
    return "yaw";
  }


  Eigen::Matrix<double,1,1> get_yaw(Eigen::Quaterniond& q)
  {
    return Eigen::Matrix<double,1,1>::Constant(
        std::atan2(2.*(q.w()*q.z() + q.x()*q.y()), 1.-2.*(q.y()*q.y() + q.z()*q.z())));
  }

  /**
   * The method called by the msf_core to apply the measurement represented by
   * this object
   */
  virtual void Apply(shared_ptr<EKFState_T> state_nonconst_new,
                     msf_core::MSF_Core<EKFState_T>& core) {

      const EKFState_T& state = *state_nonconst_new;
      // init variables
      Eigen::Matrix<double, nMeasurements,
          msf_core::MSF_Core<EKFState_T>::nErrorStatesAtCompileTime> H_old;
      Eigen::Matrix<double, nMeasurements, 1> r_old;

      H_old.setZero();
      
      if(state_nonconst_new->time == msf_core::constants::INVALID_TIME){
        MSF_WARN_STREAM(
            "Apply yaw update was called with an invalid state");
        return;
      }

      enum{
        idx_q = msf_tmp::GetStartIndex<EKFState_T::StateSequence_T,
          typename msf_tmp::GetEnumStateType<EKFState_T::StateSequence_T,
            StateDefinition_T::q>::value,
          msf_tmp::CorrectionStateLengthForType>::value,

        //idx_b_yaw = msf_tmp::GetStartIndex<EKFState_T::StateSequence_T,
        //  typename msf_tmp::GetEnumStateType<EKFState_T::StateSequence_T,
        //    StateDefinition_T::b_yaw>::value,
        //  msf_tmp::CorrectionStateLengthForType>::value
      };


      // Construct H matrix.
      // Yaw:
      H_old.block<1, 1>(0, idx_q + 2)(0) = 1;  // yaw
      // Yaw bias.
      //H_old.block<1, 1>(0, idx_b_yaw)(0) = -1;  //yaw_b

      // Construct residuals.
      // Yaw
      Eigen::Quaterniond qyaw_old = state.Get<StateDefinition_T::q>();
      Eigen::Quaterniond q_err = qyaw_old.conjugate() * z_q_;

      //r_old.block<1, 1>(0, 0) = get_yaw(z_q_) - (get_yaw(qyaw_old) - state.Get<StateDefinition_T::b_yaw>());
      r_old.block<1, 1>(0, 0) = get_yaw(q_err);

      // Call update step in base class.
      this->CalculateAndApplyCorrection(state_nonconst_new, core, H_old, r_old, R_);

  }
};

}  // namespace msf_yaw_sensor
}  // namespace msf_updates
#endif  // YAW_MEASUREMENT_HPP_
