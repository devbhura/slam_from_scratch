/// \file
/// \brief EKF Slam 
#include <armadillo>
#include "nuslam/ekf.hpp"
#include "turtlelib/rigid2d.hpp"
#include <cmath>

namespace slam
{
    ekf::ekf()
    {

    }

    ekf::ekf(int size)
    {
        obs_size = size;
        int q_len = 3+2*obs_size;
        Q = arma::eye(q_len,q_len);
        R = arma::eye(2,2);
        A = arma::zeros(q_len,q_len);
        H = arma::zeros(2,q_len);
        Sigma_previous = arma::zeros(q_len,q_len);
        Sigma_predict = arma::zeros(q_len,q_len);
        q_predict = arma::zeros(q_len,1);
        Sigma_update = arma::eye(q_len,size); 
        q_update = arma::zeros(q_len,1);
        q_previous = arma::zeros(q_len,1);
        I = arma::eye(q_len,q_len);
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

    /// \brief calculate state prediction
    arma::Mat<double> ekf::predict_q(turtlelib::Twist u)
    {
        arma::Mat<double> update;

        if(u.thetadot == 0.0)
        {
            update = {0, u.xdot*cos(q_previous(0)),  u.xdot*sin(q_previous(0)) };
        }
        else
        {
            update = {u.thetadot, (-u.xdot*sin(q_previous(0))/u.thetadot) + (u.xdot*sin((q_previous(0)+u.thetadot))/u.thetadot), (u.xdot*cos(q_previous(0))/u.thetadot) - (u.xdot*cos((q_previous(0)+u.thetadot))/u.thetadot)};
        }
        
        arma::Mat<double> update_zeros = arma::zeros(2*obs_size, 1); 
        update = join_cols(update, update_zeros); 

        q_predict = q_previous + update;

        return q_predict;
    }

    /// \brief Calculate the A matrix
    arma::Mat<double> ekf::calc_A(turtlelib::Twist u)
    {
        arma::Mat<double> A_q;
        if(u.thetadot==0.0)
        {
            A_q = {{0 ,                           0, 0},
                 {-u.xdot*sin(q_previous(0)),   0, 0},
                 {u.xdot*cos(q_previous(0)),    0, 0}};
        }
        else{
            A_q = {{0 ,                                       0, 0},
                 {(-u.xdot*cos(q_previous(0))/u.thetadot) + (u.xdot*cos((q_previous(0)+u.thetadot))/u.thetadot),    0, 0},
                 {(-u.xdot*sin(q_previous(0))/u.thetadot) + (u.xdot*sin((q_previous(0)+u.thetadot))/u.thetadot),                0, 0}};
        }

        arma::Mat<double> zeros2n_3 = arma::zeros(2*obs_size,3);
        arma::Mat<double> zeros3_2n = arma::zeros(3,2*obs_size);
        arma::Mat<double> zeros2n_2n = arma::zeros(2*obs_size, 2*obs_size); 
        A_q = join_rows(A_q, zeros3_2n); 
        A = join_cols(A_q, join_rows(zeros2n_3, zeros2n_2n)); 

        A += I;
        
        // ekf::setA(A);
        return A;
    }

    /// \brief Calculate the H matrix
    arma::Mat<double> ekf::calc_H(int j)
    {
        double mx = q_predict[2 + 2*(j)]
        arma::Mat<double> H; 
        double d = pow(delt.x,2) + pow(delt.y, 2);

        H = {{0, -delt.x/sqrt(d), -delt.y/sqrt(d)},
             {-1, delt.y/d, -delt.x/d}};

        return H; 
    }

    /// \brief set initial state and covariance matrices
    void ekf::initial_state(arma::Mat<double> x_0, arma::Mat<double> Sigma_0)
    {
        q_previous = x_0;
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


