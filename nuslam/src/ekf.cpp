/// \file
/// \brief EKF Slam 
#include <armadillo>
#include "nuslam/ekf.hpp"
#include "turtlelib/rigid2d.hpp"
#include <cmath>
#include <iostream>

namespace slam
{
    ekf::ekf()
    {

    }

    void ekf::ekf_size(int size)
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
        z_predict = arma::zeros(2,1); 
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
        arma::Mat<double> update = arma::zeros(3,1);

        if(u.thetadot == 0.0)
        {
            update(0) = 0.0;  
            update(1) = u.xdot*cos(q_previous(0)); 
            update(2) = u.xdot*sin(q_previous(0));
        }
        else
        {
            update(0) = u.thetadot;  
            update(1) = (-u.xdot*sin(q_previous(0))/u.thetadot) + (u.xdot*sin((q_previous(0)+u.thetadot))/u.thetadot); 
            update(2) = (u.xdot*cos(q_previous(0))/u.thetadot) - (u.xdot*cos((q_previous(0)+u.thetadot))/u.thetadot);
        }
        
        arma::Mat<double> update_zeros = arma::zeros(2*obs_size, 1); 
        arma::Mat<double> updated = arma::join_cols(update, update_zeros); 
        updated.print(std::cout << "updated"); 
        q_predict = q_previous + updated;

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
        A_q = arma::join_rows(A_q, zeros3_2n); 
        arma::Mat<double> zero2n_2n3 = arma::join_rows(zeros2n_3, zeros2n_2n);
        A = arma::join_cols(A_q, zero2n_2n3); 

        A += I;
        
        // ekf::setA(A);
        return A;
    }

    /// \brief Calculate the H matrix
    arma::Mat<double> ekf::calc_H(int j)
    {
        double mx = q_predict[3 + 2*j]; 
        double my = q_predict[4 + 2*j]; 
        double theta = q_predict[0]; 
        double x = q_predict[1];
        double y = q_predict[2];
        
        z_predict(0) = sqrt(pow((mx - x),2)+ pow((my-y),2));
        z_predict(1) = atan2((my-y),(mx-x)) - theta;

        double delx = mx - x; 
        double dely = my - y;

        arma::Mat<double> H_q; 
        double d = pow(delx,2) + pow(dely, 2);
        // arma::Mat<double> H_q(3,2); 
        // H_q(0,0) = 0.0;
        // H_q(1,0) = -1.0;
        // H_q(0,1) = -delx/sqrt(d);
        // H_q(1,1) = dely/d;
        // H_q()
        H_q = {{0, -delx/sqrt(d), -dely/sqrt(d)},
               {-1, dely/d, -delx/d}};
        
        arma::Mat<double> H_m = arma::zeros(2,2*obs_size); 
        int index = 2*j;
        H_m(0,index) = delx/sqrt(d);
        H_m(1,index) = -dely/d;
        H_m(0,index+1) = dely/sqrt(d);
        H_m(1,index+1) = delx/d;

        H = arma::join_rows(H_q, H_m); 

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

    arma::Mat<double> ekf::update(arma::Mat<double> z_measured)
    {
        H.print(std::cout << "H"); 
        Sigma_predict.print(std::cout<<"Sigma Pred"); 
        K = (Sigma_predict*(H.t()))*inv((H*Sigma_predict*(H.t())) + R);
        q_update = q_predict + (K*(z_measured  - z_predict));
        Sigma_update = (I - K*H)*Sigma_predict;

        q_previous = q_update; 
        Sigma_previous = Sigma_update;
        return q_previous; 
    }

}



