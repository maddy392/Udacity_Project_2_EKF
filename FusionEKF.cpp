#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#include "measurement_package.h"
#include <math.h>

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
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
  H_laser_ << 1.0,0,0,0,
        0,1.0,0,0;

  // initializing state vector; px, py = 1, also vx, vy = 1;
  //ekf_.x_ = VectorXd(4);
  //ekf_.x_ << 1,1,1,1;

  // Initializing H for radar data by referencing Calculate Jacobian function;
  //Hj_ << tools.CalculateJacobian(const VectorXd FusionEKF.ekf_.x_);

  // initializing state covariance matrix to a high number 1000
  ekf_.P_ = MatrixXd (4,4);
  ekf_.P_ << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1000, 0,
        0, 0, 0, 1000;

  // will later modify F based on timestamp in ProcessMeasurement function below;
  /*ekf_.F_ = MatrixXd (4,4);
  ekf_.F_ << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
  */

  float noise_ax = 9;
  float noise_ay = 9;

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) 
  {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    //cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 0, 0, 0, 0; // Need to play around with later


    // First timestamp recorded;
    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) 
    {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      float ro = measurement_pack.raw_measurements_[0];  // The distance from origin to the object
      float theta = measurement_pack.raw_measurements_[1]; // angle made by ro with the y axis;
      float ro_dot = measurement_pack.raw_measurements_[2]; // velocity in the direction of ro
      ekf_.x_ << ro*cos(theta), ro*sin(theta),0,0; //ro_dot*cos(theta), ro_dot*sin(theta);

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER)
    {
      /**
      Initialize state.
      */
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
      //is_initialized_ = true;
    }


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
  float dt = (measurement_pack.timestamp_ - previous_timestamp_)/1000000; // to convert microseconds to seconds 

  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;

  // Initializing and Integrating dt (time difference) into the state transition matrix;
  ekf_.F_ = MatrixXd (4,4);
  ekf_.F_ << 1, 0, dt, 0,
        0, 1, 0, dt,
        0, 0, 1, 0,
        0, 0, 0, 1;

  // Initializing process convariance matrix
  float noise_ax = 9;
  float noise_ay = 9;
  ekf_.Q_  = MatrixXd(4,4);
  ekf_.Q_ <<  dt_4/4 *noise_ax, 0, dt_3/2*noise_ax, 0,
        0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
        dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
        0, dt_3/2*noise_ay, 0, dt_2*noise_ay;

  //cout << "call predict"<<endl;

  ekf_.Predict();


  //cout << "finish predict"<<endl;

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
    //ekf_.R_ = R_radar_; // Line 32;

    //printf("%s %d\n",__FILE__,__LINE__);

    //ekf_.H_ = tools.CalculateJacobian(ekf_.x_);

    //printf("%s %d\n",__FILE__,__LINE__);

    // calling UpdateEKF() function which is defined for update for radar measurements;

    //cout << "H_" << endl << ekf_.H_ << endl << endl;

    //printf("%s %d\n",__FILE__,__LINE__);

    //ekf_.UpdateEKF(measurement_pack.raw_measurements_);

    //printf("%s %d\n",__FILE__,__LINE__);

  } else {
    // Laser updates

    ekf_.H_ = H_laser_; // defined in line 41;
    ekf_.R_ = R_laser_; // Given in line 28;

    //printf("%s %d\n",__FILE__,__LINE__);

    // calling Update() function which is defined for update for lidar measurements;
    ekf_.Update(measurement_pack.raw_measurements_);

    //printf("%s %d\n",__FILE__,__LINE__);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
