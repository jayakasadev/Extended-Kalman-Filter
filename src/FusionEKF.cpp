#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * initializes the filter, calls the predict function, calls the update function
 */


/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
    // cout << "FusionEKF::FusionEKF" << endl;
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
      TODO:
        * Finish initializing the FusionEKF.
        * Set the process and measurement noises
      */

    // for lidar
    H_laser_ << 1, 0, 0, 0,
                 0, 1, 0, 0;

    // jacobian for radar
    Hj_ << 1, 1, 0, 0,
            1, 1, 0, 0,
            1, 1, 1, 1;

    // initialize the covariance matrices
    // can be done here or in the process measurement method

    //the initial transition matrix F_
    ekf_.F_ = MatrixXd(4, 4);
    ekf_.F_ << 1, 0, 1, 0,
                0, 1, 0, 1,
                0, 0, 1, 0,
                0, 0, 0, 1;

    //state covariance matrix P
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ << 1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1000, 0,
                0, 0, 0, 1000;

    /*
    cout << "R_laser_: " << endl;
    cout << R_laser_ << endl;
    cout << "\nR_radar_:" << endl;
    cout << R_radar_ << endl;

    cout << "H_laser_: " << endl;
    cout << H_laser_ << endl;
    cout << "\nHj_:" << endl;
    cout << Hj_ << endl;

    cout << "\nekf_.F_: " << endl;
    cout << ekf_.F_ << endl;
    cout << "\nekf_.P_:" << endl;
    cout << ekf_.P_ << endl;
    cout << "\n" << endl;
     */
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
    // cout << "FusionEKF::ProcessMeasurement" << endl;

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    // cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        // cout << "RADAR" << endl;
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
       range, bearing, rate -> px, py, vx, vy
      */

        // cout << "RADAR: X vector " << measurement_pack.raw_measurements_ << endl;

        float rho = measurement_pack.raw_measurements_(0);
        float phi = measurement_pack.raw_measurements_(1);
        float dot = measurement_pack.raw_measurements_(2);

        ekf_.x_(0) = rho * cos(phi);
        ekf_.x_(1) = rho * sin(phi);
        ekf_.x_(2) = dot * cos(phi);
        ekf_.x_(3) = dot * sin(phi);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
        // cout << "LASER: X vector" << measurement_pack.raw_measurements_ << endl;
      /**
      Initialize state.
      */
        // cout << measurement_pack.raw_measurements_ << endl;
        ekf_.x_(0) = measurement_pack.raw_measurements_(0);
        ekf_.x_(1) = measurement_pack.raw_measurements_(1);
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
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

    //compute the time elapsed between the current and previous measurements
    float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
    previous_timestamp_ = measurement_pack.timestamp_;


    float d2 = dt   * dt;
    float d3 = d2 * dt;
    float d4 = d3 * dt;

    //Modify the F matrix so that the time is integrated
    ekf_.F_(0, 2) = dt;
    ekf_.F_(1, 3) = dt;

    //set the acceleration noise components
    float noise_ax = 9;
    float noise_ay = 9;

    //set the process covariance matrix Q
    ekf_.Q_ = MatrixXd(4, 4);
    ekf_.Q_ << d4 / 4 * noise_ax, 0, d3 / 2 * noise_ax, 0,
                0, d4 / 4 * noise_ay, 0, d3 / 2 * noise_ay,
                d3 / 2 * noise_ax, 0, d2*noise_ax, 0,
                0, d3 / 2 * noise_ay, 0, d2*noise_ay;
    // cout << "ekf_.Q_:\n" << endl;
    // cout << ekf_.Q_ << endl;

    // cout << "Calling Predict IN FusionEKF::ProcessMeasurement" << endl;
  ekf_.Predict();
    // cout << "BACK IN FusionEKF::ProcessMeasurement" << endl;

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // cout << "RADAR UPDATE: " << ekf_.x_ << endl;
    // Radar updates
      Hj_ = tools.CalculateJacobian(ekf_.x_);
      ekf_.H_ = Hj_;
      ekf_.R_ = R_radar_;

      // cout << "Calling UpdateEKF IN FusionEKF::ProcessMeasurement" << endl;
      ekf_.UpdateEKF(measurement_pack.raw_measurements_);
      // cout << "BACK IN FusionEKF::ProcessMeasurement" << endl;
  } else {
      // cout << "LASER UPDATE: " << ekf_.x_ << endl;
    // Laser updates
      ekf_.H_ = H_laser_;
      ekf_.R_ = R_laser_;

      // cout << "Calling Update IN FusionEKF::ProcessMeasurement" << endl;
      ekf_.Update(measurement_pack.raw_measurements_);
      // cout << "BACK IN FusionEKF::ProcessMeasurement" << endl;
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
