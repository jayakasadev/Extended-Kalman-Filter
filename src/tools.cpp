#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

/**
 * unction to calculate RMSE and the Jacobian matrix
 */

Tools::Tools() {}

Tools::~Tools() {}

/**
 * Method calculates the Root Mean Squared Error value
 *
 * @param estimations
 * @param ground_truth
 * @return
 */
VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth) {
    // cout << "Tools::CalculateRMSE" << endl;
    VectorXd rmse(4);
    rmse << 0, 0, 0, 0;
    // cout << "rmse: " << endl;
    // cout << rmse << endl;

    // if there are no estimations or the number of estimations do not match the number of ground truths, return empty
    // rmse
    if(estimations.size() == 0 || estimations.size() != ground_truth.size()){
        cout << "\n\tError: Invalid estimation or ground_truth data" << endl;
        return rmse;
    }

    // accumulate squared residuals
    for(unsigned int  i = 0; i < estimations.size(); i++){
        // getting the difference between the calculated and expected
        /*
        cout << "estimations[" << i << "]: ";
        cout << estimations[i].size() << endl;
        cout << "ground_truth[" << i << "]: ";
        cout << ground_truth[i].size() << endl;
        cout << "residual: " << endl;
        cout << estimations[i] - ground_truth[i] << endl;
         */
        VectorXd residual = estimations[i] - ground_truth[i];

        // squaring the result
        // adding to rmse
        //cout << "residual: " << endl;
        //cout << residual << endl;

        residual = residual.array().square();

        //cout << "residual: " << endl;
        //cout << residual << endl;

        rmse += residual;
        //cout << "rmse: " << endl;
        //cout << rmse << endl;
    }
    // averaging over the number of estimates and returning the square root
    rmse = (rmse / estimations.size()).array().sqrt();

    cout << rmse << endl;
    return rmse;
}

/**
 * Method to calculate the Jacobian Matrix for a given Matrix
 *
 * @param x_state
 * @return
 */
MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
    // cout << "Tools::CalculateJacobian" << endl;
    MatrixXd jacobian(3, 4);

    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    // getting the basic calculations done here
    float calc1 = pow(px, 2) + pow(py, 2);

    // check for division by zero
    // TODO create an external class or other definition that contains all hyperparameters
    if(fabs(calc1) < 0.0001){
        cout << "\n\tError: Division by Zero" << endl;
        return jacobian;
    }


    float calc2 = sqrt(calc1);
    float calc3 = calc1 * calc2;
    float calc4 = vx * py;
    float calc5 = vy * px;

    // compute the jacobian matrix
    jacobian << px / calc2, py / calc2, 0, 0,
                - px / calc1, px / calc1, 0, 0,
            (py * (calc4 - calc5)) / calc3, (px * (calc5 - calc4)) / calc3, px / calc2, py / calc2;

    // cout << jacobian << endl;
    return jacobian;
}
