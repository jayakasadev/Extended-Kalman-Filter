#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * defines the predict function, the update function for lidar, and the update function for radar
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

/**
 * Prediction Function
 */
void KalmanFilter::Predict() {
    // cout << "KalmanFilter::Predict" << endl;
    // calculating the prediction vector -> x'
    x_ = F_ * x_;

    // uncertainty function
    // calculating the uncertainty vector -> P
    MatrixXd FT = F_.transpose();
    P_ = F_ * P_ * FT + Q_;
}

/**
 * Update Function for Lidar with Kalman Filter
 *
 * @param z
 */
void KalmanFilter::Update(const VectorXd &z) {
    // cout << "KalmanFilter::Update" << endl;

    // for lidar or laser measurements

    // comparison function
    VectorXd y = z - H_ * x_;
    // cout << "y: " << y << endl;

    // calling the kalman filter common equations
    // cout << "Calling KF" << y << endl;
    KF(y);
}

/**
 * Update Function for Radar with Extended Kalman Filter
 *
 * @param z
 */
void KalmanFilter::UpdateEKF(const VectorXd &z) {

    // cout << "KalmanFilter::UpdateEKF: "<< z << " " << z.size() << endl;

    // float px = z(0);
    // float py = z(1);
    // float vx = z(2);
    // float vy = z(3);

    // cout << px << " " << py << " " << vx << " " << vy << endl;

    // float rho = sqrt(pow(px, 2) + pow(py, 2));
    // float phi = atan2(py, px);
    // float dot;

    float rho = z(0);
    float phi = z(1);
    float dot = z(2);

    // cout << rho << " " << phi << " " << dot << endl;

    /*
    if(fabs(rho) < 0.0001){
        // the vehicle did not move
        dot = 0;
    }
    else{
        dot = (px * vx + py * vy) / rho;
    }
    cout << dot << endl;
     */

    VectorXd h(3);
    h << rho, phi, dot;

    // cout << "h: " << h << endl;

    VectorXd y = z - h;

    // cout << "y: " << y << endl;

    // calling the kalman filter common equations
    // cout << "Calling KF" << y << endl;
    KF(y);
}

/**
 * Common code for Kalman Filter and Extended Kalman Filter
 *
 * @param z
 */
void KalmanFilter::KF(const VectorXd &y) {

    // cout << "KalmanFilter::KF" << endl;

    // H transpose
    MatrixXd HT = H_.transpose();

    // project system uncertainty into measurement space
    // measurement projection function + R
    MatrixXd S = H_ * P_ * HT + R_;

    // S Inverse
    MatrixXd SI = S.inverse();

    // Kalman Filter gain
    MatrixXd K = P_ * HT * SI;

    // x_ estimate
    x_ = x_ + K * y;

    // Identity matrix
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);

    // uncertainty matrix
    P_ = (I - K * H_) * P_;
}