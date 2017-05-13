#include "FusionEKF.h"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
      0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
      0, 0.0009, 0,
      0, 0, 0.09;

  /**
  TODOdone:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */

  // y = z - H * x', where z is the sensor data and H maps prediction into sensor space

  H_laser_ <<
           1, 0, 0, 0,
      0, 1, 0, 0;

  // Hj will need to change in accordance to the Jacobian
//  Hj_ <<
//      0,              0,              0,              0,
//      0,              0,              0,              0,
//      0,              0,              0,              0;

  //set the acceleration noise components
  noise_ax = 9;
  noise_ay = 9;

  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, 1000, 0,
      0, 0, 0, 1000;

  ekf_.F_ = MatrixXd::Zero(4, 4);
  ekf_.Q_ = MatrixXd::Zero(4, 4);

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODOdone:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      auto rho = measurement_pack.raw_measurements_[0];
      auto phi = measurement_pack.raw_measurements_[1];

      ekf_.x_ << rho * cos(phi), rho * sin(phi), 0, 0;

    } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
    }

    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODOdone:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */



  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;

  cout << "dt: " << dt << "\n" << endl;

  ekf_.F_(0, 0) = 1;
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 1) = 1;
  ekf_.F_(1, 3) = dt;

  float dt4_4 = pow(dt, 4) / 4;
  float dt3_2 = pow(dt, 3) / 2;
  float dt2 = dt * dt;

  ekf_.Q_(0, 0) = dt4_4 * noise_ax;
  ekf_.Q_(0, 2) = dt3_2 * noise_ax;
  ekf_.Q_(1, 1) = dt4_4 * noise_ay;
  ekf_.Q_(1, 3) = dt3_2 * noise_ay;

  ekf_.Q_(2, 0) = dt3_2 * noise_ax;
  ekf_.Q_(2, 2) = dt2 * noise_ax;
  ekf_.Q_(3, 1) = dt3_2 * noise_ay;
  ekf_.Q_(3, 3) = dt2 * noise_ay;

  ekf_.Predict();
  cout << ekf_.x_ << ": x\n" << endl;
  cout << ekf_.P_ << ": P\n" << endl;


  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    // Update ekf_
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    cout << "HJ_: " << Hj_ << "\n" << endl;
    ekf_.H_ = Hj_;
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    // Laser updates
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  previous_timestamp_ = measurement_pack.timestamp_;
  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
