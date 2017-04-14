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
#ifndef VISION_XY_MEASUREMENT_HPP_
#define VISION_XY_MEASUREMENT_HPP_

#include <msf_core/msf_measurement.h>
#include <msf_core/msf_core.h>
#include <msf_core/eigen_utils.h>

namespace msf_updates {
namespace vision_xy_measurement {
enum {
  nMeasurements = 3
};

/**
 * \brief A measurement as provided by a vision sensor XY yaw
 */
typedef msf_core::MSF_Measurement<
    geometry_msgs::PoseStamped,
    Eigen::Matrix<double, nMeasurements, nMeasurements>, msf_updates::EKFState> VisionXYMeasurementBase;
struct VisionXYMeasurement : public VisionXYMeasurementBase {
 private:
  typedef VisionXYMeasurementBase Measurement_t;
  typedef Measurement_t::Measurement_ptr measptr_t;

  virtual void MakeFromSensorReadingImpl(measptr_t msg) {

    Eigen::Matrix<double, nMeasurements,
        msf_core::MSF_Core<msf_updates::EKFState>::nErrorStatesAtCompileTime> H_old;
    Eigen::Matrix<double, nMeasurements, 1> r_old;

    H_old.setZero();

    // Get measurement.
    z_p_ = Eigen::Matrix<double, 2, 1>(msg->pose.position.x, 
                                       msg->pose.position.y);

    z_q_ = Eigen::Quaterniond(msg->pose.orientation.w,
                             msg->pose.orientation.x,
                             msg->pose.orientation.y,
                             msg->pose.orientation.z);

    const double s_zp = n_zp_ * n_zp_;
    const double s_zq = n_zq_ * n_zq_;
    R_ = (Eigen::Matrix<double, nMeasurements, 1>() << s_zp, s_zp, s_zq).finished().asDiagonal();

  }
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Matrix<double, 2, 1> z_p_;  /// Position xy measurement.
  double n_zp_;  /// Position xy measurement noise.

  Eigen::Quaterniond z_q_;
  double n_zq_;

  int fixedstates_;

  typedef msf_updates::EKFState EKFState_T;
  typedef EKFState_T::StateSequence_T StateSequence_T;
  typedef EKFState_T::StateDefinition_T StateDefinition_T;
  virtual ~VisionXYMeasurement() {
  }
  VisionXYMeasurement(double n_zp, double n_zq,
                      bool isabsoluteMeasurement, int sensorID, int fixedstates,
                      bool enable_mah_outlier_rejection, double mah_threshold)
      : VisionXYMeasurementBase(isabsoluteMeasurement, sensorID,
                                enable_mah_outlier_rejection, mah_threshold),
        n_zp_(n_zp),
        n_zq_(n_zq),
        fixedstates_(fixedstates) {
  }
  virtual std::string Type() {
    return "vision_xy";
  }

  Eigen::Matrix<double,1,1> get_yaw(Eigen::Quaterniond& q)
  {
    return Eigen::Matrix<double,1,1>::Constant(
        std::atan2(2.*(q.w()*q.z() + q.x()*q.y()), 1.-2.*(q.y()*q.y() + q.z()*q.z())));
  }

  virtual void CalculateH(
      shared_ptr<EKFState_T> state_in,
      Eigen::Matrix<double, nMeasurements,
          msf_core::MSF_Core<EKFState_T>::nErrorStatesAtCompileTime>& H) {
    const EKFState_T& state = *state_in;  // Get a const ref, so we can read core states.

    H.setZero();

    // Get rotation matrices.
    Eigen::Matrix<double, 3, 3> C_q = state.Get<StateDefinition_T::q>()
        .conjugate().toRotationMatrix();

    // Preprocess for elements in H matrix.
    Eigen::Matrix<double, 3, 3> p_prism_imu_sk = Skew(
        state.Get<StateDefinition_T::p_ip>());

    // Get indices of states in error vector.
    enum {
      idxstartcorr_p_ = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
          StateDefinition_T::p>::value,
      idxstartcorr_v_ = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
          StateDefinition_T::v>::value,
      idxstartcorr_q_ = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
          StateDefinition_T::q>::value,
      idxstartcorr_p_pi_ = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
          StateDefinition_T::p_ip>::value,
      idxstartcorr_b_yaw = msf_tmp::GetStartIndexInCorrection<StateSequence_T,
          StateDefinition_T::b_yaw>::value
    };

    bool fixed_p_pos_imu = (fixedstates_ & 1 << StateDefinition_T::p_ip);

    // Clear crosscorrelations.
    if (fixed_p_pos_imu)
      state_in->ClearCrossCov<StateDefinition_T::p_ip>();

    // Construct H matrix:
    // Position:
    H.block<2, 2>(0, idxstartcorr_p_) = Eigen::Matrix<double, 2, 2>::Identity();  // p xy

    H.block<3, 3>(0, idxstartcorr_q_) = -C_q.transpose() * p_prism_imu_sk;  // q

    H.block<3, 3>(0, idxstartcorr_p_pi_) =
        fixed_p_pos_imu ?
            Eigen::Matrix<double, 3, 3>::Zero() : (C_q.transpose()).eval();  //p_pos_imu_

    H.block<1, 1>(3, idxstartcorr_q_ + 2) (0) = 1.0;
    H.block<1, 1>(3, idxstartcorr_b_yaw)(0) = -1.0;  //yaw_b

  }

  /**
   * The method called by the msf_core to apply the measurement represented by this object.
   */
  virtual void Apply(shared_ptr<EKFState_T> state_nonconst_new,
                     msf_core::MSF_Core<EKFState_T>& core) {

    if (isabsolute_) {  // Does this measurement refer to an absolute measurement,
      // or is it relative to the last measurement.
      // Get a const ref, so we can read core states.
      const EKFState_T& state = *state_nonconst_new;
      // init variables
      Eigen::Matrix<double, nMeasurements,
          msf_core::MSF_Core<EKFState_T>::nErrorStatesAtCompileTime> H_new;
      Eigen::Matrix<double, nMeasurements, 1> r_old;

      CalculateH(state_nonconst_new, H_new);

      // Get rotation matrices.
      Eigen::Matrix<double, 3, 3> C_q = state.Get<StateDefinition_T::q>()
          .conjugate().toRotationMatrix();

      // Construct residuals:
      // Position
      r_old.block<2, 1>(0, 0) = z_p_
          - (state.Get<StateDefinition_T::p>()
              + C_q.transpose() * state.Get<StateDefinition_T::p_ip>()).block<2, 2>(0, 0);

      Eigen::Quaterniond qyaw_old = Eigen::Quaterniond(state.Get<StateDefinition_T::q>());
      Eigen::Quaterniond qerr = qyaw_old.conjugate() * z_q_;

      r_old.block<1, 1>(2, 0) = get_yaw(qerr);

      if (!CheckForNumeric(r_old, "r_old")) {
        MSF_ERROR_STREAM("r_old: "<<r_old);
        MSF_WARN_STREAM(
            "state: "<<const_cast<EKFState_T&>(state). ToEigenVector().transpose());
      }
      if (!CheckForNumeric(H_new, "H_old")) {
        MSF_ERROR_STREAM("H_old: "<<H_new);
        MSF_WARN_STREAM(
            "state: "<<const_cast<EKFState_T&>(state). ToEigenVector().transpose());
      }
      if (!CheckForNumeric(R_, "R_")) {
        MSF_ERROR_STREAM("R_: "<<R_);
        MSF_WARN_STREAM(
            "state: "<<const_cast<EKFState_T&>(state). ToEigenVector().transpose());
      }

      // Call update step in base class.
      this->CalculateAndApplyCorrection(state_nonconst_new, core, H_new, r_old,
                                        R_);
    } else {
      MSF_ERROR_STREAM_THROTTLE(
          1, "You chose to apply the vision measurement "
          "as a relative quantitiy, which is currently not implemented.");
    }
  }
};
}  // namespace vision_xy_measeurement
}  // namespace msf_updates

#endif  // VISION_XY_MEASUREMENT_HPP_
