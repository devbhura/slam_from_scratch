/// \file
/// \brief EKF Slam 
#include <armadillo>
#include "nuslam/ekf.hpp"

namespace ekf
{

    ekf::ekf()
    {
        Q = arma::eye(3,3);
        R = arma::eye(2,2);
        A = arma::zeros(3,3);
        H = arma::zeros(2,3);
        Sigma_previous = arma::zeros(3,3);
        Sigma_predict = arma::zeros(3,3);
        q_predict = arma::zeros(3,1);
        Sigma_update = arma::eye(3,3); 
        q_update = arma::zeros(3,3);
    }

    /// \brief Set the Q matrix
    void ekf::setQ(arma::Mat<double> Q)
    {
        Q = Q;
    }

    /// \brief Set the R matrix
    void ekf::setR(arma::Mat<double> R)
    {
        R = R;
    }

    /// \brief Set the A matrix
    void ekf::setA(arma::Mat<double> A)
    {
        A = A;
    }

    /// \brief Set the A matrix
    void ekf::setH(arma::Mat<double> H)
    {
        H = H;
    }

    /// \brief set initial state and covariance matrices
    void ekf::initial_state(arma::Mat<double> x_0, arma::Mat<double> Sigma_0)
    {
        q_predict = x_0;
        Sigma_previous = Sigma_0;
    }

    void ekf::predict()
    {
        Sigma_predict = A*Sigma_previous*(A.t()) + Q;
    }

    void ekf::update(arma::Mat<double> z_measured, arma::Mat<double> z_predict)
    {
        K = (Sigma_predict*(H.t()))*inv(H*Sigma_previous*(H.t()) + R);
        q_update = q_predict + K*(z_measured  - z_predict);
        Sigma_update = (I - K*H)*Sigma_predict;

        
        Sigma_previous = Sigma_update;
    }

}



