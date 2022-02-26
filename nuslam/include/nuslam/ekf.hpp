#ifndef EKF_INCLUDE_GUARD_HPP
#define EKF_INCLUDE_GUARD_HPP

#include <math.h>
#include <armadillo>
/// \file
/// \brief launches turtlebot3 and obstacles in rviz scene for visualization


namespace ekf
{

    class ekf
    {
        private:

            arma::Mat<double> Q = arma::eye(3,3);
            arma::Mat<double> R = arma::eye(2,2);
            arma::Mat<double> A = arma::zeros(3,3);
            arma::Mat<double> H = arma::zeros(2,3);
            arma::Mat<double> Sigma_previous = arma::zeros(3,3);
            arma::Mat<double> Sigma_predict = arma::zeros(3,3);

            arma::Mat<double> q_predict = arma::zeros(3,1);
            arma::Mat<double> Sigma_update; 
            arma::Mat<double> q_update;
            arma::Mat<double> K;



        public:

            /// \brief Initialize an ekf model 
            ekf();

            /// \brief Set the Q matrix
            void setQ(arma::Mat<double> Q);

            /// \brief Set the R matrix
            void setR(arma::Mat<double> R);

            /// \brief Set the A matrix
            void setA(arma::Mat<double> A);

            /// \brief Set the A matrix
            void setH(arma::Mat<double> H);

            /// \brief set initial state and covariance matrices
            void initial_state(arma::Mat<double> x_0, arma::Mat<double> Sigma_0);

            void predict();
            void update(arma::Mat<double> z_measured, arma::Mat<double> z_predict);

            arma::mat

            

    }

}



# endif
