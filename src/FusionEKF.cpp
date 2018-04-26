#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

//bool is_initialized_;
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
  //ekf_.F_=MatrixXd(4,4);
  ekf_.P_=MatrixXd(4,4);
    
  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  //measurement matrix - laser
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;
  
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
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 0.0,0.0,0.0,0.0;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      cout << "Got me some RADAR data" << endl;
      //read radar data
      float ro = measurement_pack.raw_measurements_(0);
      float theta = measurement_pack.raw_measurements_(1);
      float ro_dot = measurement_pack.raw_measurements_(2);

      //convert polar coordinates to cartesian
      float x=ro*cos(theta);
      float y=ro*sin(theta);
      float vx=ro_dot*cos(theta);
      float vy=ro_dot*cos(theta);
      
      //fill state matrix
      ekf_.x_ << x,y,vx,vy;
      
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      cout << "Got me some LASER data" << endl;
      //read lidar data
      float x=measurement_pack.raw_measurements_(0);
      float y=measurement_pack.raw_measurements_(1);

      //fill state matrix - only fill first to values as lidar has no associated      
      //veloicty data
      ekf_.x_(0)=x;
      ekf_.x_(1)=y;
      
    }
    //Initialise ekf_.P_ the covarianvce matrix
    ekf_.P_ << 1,0,0,0,
               0,1,0,0,
               0,0,1000,0,
               0,0,0,1000;
    // store timestamp
    previous_timestamp_ = measurement_pack.timestamp_;
    // done initializing, no need to predict or update
    cout<< "Boom you is initialised" << endl;
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
  
  //convert time from millieseconds to seconds
  float dt = (measurement_pack.timestamp_ - previous_timestamp_)/1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  //cout<< "dt: " << dt << endl;
  // State transition matrix update
  ekf_.F_ = MatrixXd(4, 4);

  ekf_.F_ << 1, 0, dt, 0,
         		 0, 1, 0, dt,
			       0, 0, 1, 0,
			       0, 0, 0, 1;
  // Noise covariance matrix computation	
  // Noise values from the course notes
  float noise_ax = 9.0;
  float noise_ay = 9.0;

  //calc powers of time variable to reduce reperated computation
  float dt_2 = dt*dt;
  float dt_3 = dt_2*dt;
  float dt_4 = dt_3*dt;

  //  Calculate process covarience matrix
  ekf_.Q_ = MatrixXd(4,4);
  ekf_.Q_<< (dt_4/4)*noise_ax, 0, (dt_3/2)*noise_ax,0,
            0,(dt_4/4)*noise_ay,0,(dt_3/2)*noise_ay,
            (dt_3/2)*noise_ax,0,dt_2*noise_ax,0,
            0,(dt_3/2)*noise_ay,0,dt_2*noise_ay;
            
  ekf_.Predict();

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
    cout << "Got me some RADAR data" << endl;
    /* The H matrix projects our belief of the objects current state in to the  
    measurement space of the sensor, because we are working in carteisan space 
    but the radar is working in polar space we need to perform a non-linear
    transformation and this requires us to calculate the jacobian matrix
    */
    ekf_.H_= tools.CalculateJacobian(ekf_.x_);
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    // Laser updates
    cout << "Got me some LIDAR data" << endl;
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "============================"<< endl;
  //cout << "x = " << ekf_.x_ << endl;
  //cout << "y = " << ekf_.x_(1) << endl;
  //cout << "P_ = " << ekf_.P_ << endl;
}
