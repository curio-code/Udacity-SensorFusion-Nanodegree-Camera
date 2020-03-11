#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = false;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);
  x_.fill(0.0);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);
  P_ << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0,
        0, 0, 1, 0, 0,
        0, 0, 0, 1, 0,
        0, 0, 0, 0, 1;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1.5;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 2;
  
  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  
  /**
   * End DO NOT MODIFY section for measurement noise values 
   */
  
  /**
   * TODO: Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */

  is_initialized_ = false;

  n_x_ = 5;

  n_aug_ = 7;

  lambda_ = 3 - n_x_;

  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);  
  Xsig_pred_.fill(0.0);

  weights_ = VectorXd(2 * n_aug_ + 1);
  weights_.fill(0.5 / (lambda_ + n_aug_));
  weights_(0) = lambda_ / (lambda_ + n_aug_);

  time_us_ = 0.0;

}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */

  if (!is_initialized_)
  {
    if (meas_package.sensor_type_ == MeasurementPackage::LASER)
    {
      
      x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0, 0, 0;
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
    {

      double rho = meas_package.raw_measurements_[0];
      double phi = meas_package.raw_measurements_[1];
      double rhod = meas_package.raw_measurements_[2];

      x_ << rho * cos(phi), rho * sin(phi), 0, 0, 0;
    }

    time_us_ = meas_package.timestamp_;
    is_initialized_ = true;

    
    std:: cout << x_ << std::endl;
    std:: cout << "-------initialized--------" << std::endl;

    return;
  }


  double delta_t = static_cast<double>((meas_package.timestamp_ - time_us_) * 1e-6);
  time_us_ = meas_package.timestamp_;


  // Prediction
  Prediction(delta_t);
  //std::cout << "----Prediction----" << std::endl;


  // Update Measurement
  if (meas_package.sensor_type_ == MeasurementPackage::LASER)
  { 
    UpdateLidar(meas_package);
  }
  else if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
  {
    UpdateRadar(meas_package);
  }
}


void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */
   
   // Sigma Points Generation
   VectorXd x_aug = VectorXd(n_aug_);

   MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

   MatrixXd X_sig = MatrixXd(n_aug_, 2*n_aug_+1);

   x_aug.head(5) = x_;
   x_aug(5)=0.0;
   x_aug(6)=0.0;

   X_sig.col(0) = x_aug;

   P_aug.fill(0.0);
   P_aug.topLeftCorner(5,5)= P_;
   P_aug(5,5) = std_a_*std_a_;
   P_aug(6,6) = std_yawdd_*std_yawdd_;

   MatrixXd L = P_aug.llt().matrixL();

   for (int i = 0; i < n_aug_; ++i){
    X_sig.col(i + 1)          = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
    X_sig.col(i + 1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * L.col(i);
  }

  //Predict Sigma Points
    for (int i = 0; i < 2 * n_aug_ + 1; ++i)
  {
    // extract state
    double p_x = X_sig(0, i);
    double p_y = X_sig(1, i);
    double v = X_sig(2, i);
    double yaw = X_sig(3, i);
    double yawd = X_sig(4, i);
    double nu_a = X_sig(5, i);
    double nu_yawdd = X_sig(6, i);

    // predict state
    double px_p, py_p, v_p, yaw_p, yawd_p;
    if (fabs(yawd) > 0.001)
    {
      // curve path
      px_p = p_x + v/yawd*(sin(yaw+yawd*delta_t) - sin(yaw)) + 0.5*delta_t*delta_t*cos(yaw)*nu_a;
      py_p = p_y + v/yawd*(-cos(yaw+yawd*delta_t) + cos(yaw)) + 0.5*delta_t*delta_t*sin(yaw)*nu_a;
      v_p = v + delta_t*nu_a;
      yaw_p = yaw + yawd*delta_t + 0.5*delta_t*delta_t*nu_yawdd;
      yawd_p = yawd + delta_t*nu_yawdd;
    }
    else
    {
      // straight path
      px_p = p_x + v*cos(yaw)*delta_t + 0.5*delta_t*delta_t*cos(yaw)*nu_a;
      py_p = p_y + v*sin(yaw)*delta_t + 0.5*delta_t*delta_t*sin(yaw)*nu_a;
      v_p = v + delta_t*nu_a;
      yaw_p = yaw + yawd*delta_t + 0.5*delta_t*delta_t*nu_yawdd;
      yawd_p = yawd + delta_t*nu_yawdd;
    }

    // update Xsig_pred
    Xsig_pred_(0, i) = px_p;
    Xsig_pred_(1, i) = py_p;
    Xsig_pred_(2, i) = v_p;
    Xsig_pred_(3, i) = yaw_p;
    Xsig_pred_(4, i) = yawd_p;
  } 

  //Predict mean and covariance
  VectorXd final_x = VectorXd::Zero(5);
  MatrixXd final_P = MatrixXd::Zero(5, 5);

  for (int i = 0; i < 2 * n_aug_ + 1; ++i){
    final_x += weights_(i) * Xsig_pred_.col(i);
  }

  for (int i = 0; i < 2 * n_aug_ + 1; ++i){
    VectorXd x_diff = Xsig_pred_.col(i) - final_x;

    while (x_diff(3) > M_PI) x_diff(3) -= 2.0*M_PI;
    while (x_diff(3) < -M_PI) x_diff(3) += 2.0*M_PI;

    final_P += weights_(i) * x_diff * x_diff.transpose();
  }


  // write result
  x_ = final_x;
  P_ = final_P;

}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */
  
  int n_z_ = 2; //measurement state plane 
  MatrixXd Zsig = MatrixXd(n_z_, 2*n_aug_+1);//Sigma points projected in measurement Space

  VectorXd z_pred =VectorXd(n_z_);//mean of the predicted measurement

  MatrixXd S = MatrixXd(n_z_, n_z_);

  for (int i =0; i< 2*n_aug_+1;++i){
    double px = Xsig_pred_(0,i);
    double py = Xsig_pred_(1,i);
    double v = Xsig_pred_(2,i);
    double phi = Xsig_pred_(3,i);
    double phid = Xsig_pred_(4,i);

    Zsig(0,i) = px;
    Zsig(1,i) = py;
  }

  z_pred.fill(0.0);
  for (int i=0; i<2*n_aug_+1; ++i){
    z_pred = z_pred + weights_(i)*Zsig.col(i);
  }

  S.fill(0.0);
  for (int i=0; i<2*n_aug_+1; ++i){
    VectorXd z_diff = Zsig.col(i)-z_pred;

    while (z_diff(1)>M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S = S + weights_(i)*z_diff*z_diff.transpose();
  }
   MatrixXd R = MatrixXd(n_z_,n_z_);
   R<< std_laspx_*std_laspx_,0,
        0,std_laspy_*std_laspy_;
  S = S + R;
  
  MatrixXd Tc = MatrixXd(n_x_,n_z_);
  Tc.fill(0.0);
  for (int i=0; i<2*n_aug_+1; ++i){
    VectorXd z_diff = Zsig.col(i)-z_pred;

    while (z_diff(1)>M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    VectorXd x_diff = Xsig_pred_.col(i)-x_;

    while (x_diff(1)>M_PI) x_diff(1)-=2.*M_PI;
    while (x_diff(1)<-M_PI) x_diff(1)+=2.*M_PI;

    Tc = Tc + weights_(i)*x_diff*z_diff.transpose();
  }

  MatrixXd K = Tc *S.inverse();
  VectorXd z=meas_package.raw_measurements_;
  VectorXd z_diff = z-z_pred;

  while (z_diff(1)>M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  x_ =x_ + K*z_diff;
  P_ = P_ - K*S*K.transpose();
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */

    int n_z_ = 3; //measurement state plane 
  MatrixXd Zsig = MatrixXd(n_z_, 2*n_aug_+1);//Sigma points projected in measurement Space

  VectorXd z_pred =VectorXd(n_z_);//mean of the predicted measurement

  MatrixXd S = MatrixXd(n_z_, n_z_);

  for (int i =0; i< 2*n_aug_+1;++i){
    double px = Xsig_pred_(0,i);
    double py = Xsig_pred_(1,i);
    double v = Xsig_pred_(2,i);
    double phi = Xsig_pred_(3,i);
    double phid = Xsig_pred_(4,i);
    double v1 = cos(phi)*v;
    double v2 = sin(phi)*v;

    Zsig(0,i) = sqrt(px*px +py*py);
    Zsig(1,i) = atan2(py,px);
    Zsig(2,i) = (px*v1 + py*v2)/ sqrt(px*px + py*py);
  }

  z_pred.fill(0.0);
  for (int i=0; i<2*n_aug_+1; ++i){
    z_pred = z_pred + weights_(i)*Zsig.col(i);
  }

  S.fill(0.0);
  for (int i=0; i<2*n_aug_+1; ++i){
    VectorXd z_diff = Zsig.col(i)-z_pred;

    while (z_diff(1)>M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S = S + weights_(i)*z_diff*z_diff.transpose();
  }
   MatrixXd R = MatrixXd(n_z_,n_z_);
   R<< std_radr_*std_radr_,0,0,
        0,std_radphi_*std_radphi_,0,
        0,0,std_radrd_*std_radrd_;
  S = S + R;
  
  MatrixXd Tc = MatrixXd(n_x_,n_z_);
  Tc.fill(0.0);
  for (int i=0; i<2*n_aug_+1; ++i){
    VectorXd z_diff = Zsig.col(i)-z_pred;

    while (z_diff(1)>M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    VectorXd x_diff = Xsig_pred_.col(i)-x_;

    while (x_diff(1)>M_PI) x_diff(1)-=2.*M_PI;
    while (x_diff(1)<-M_PI) x_diff(1)+=2.*M_PI;

    Tc = Tc + weights_(i)*x_diff*z_diff.transpose();
  }

  MatrixXd K = Tc *S.inverse();
  VectorXd z=meas_package.raw_measurements_;
  VectorXd z_diff = z-z_pred;

  while (z_diff(1)>M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  x_ =x_ + K*z_diff;
  P_ = P_ - K*S*K.transpose();
}