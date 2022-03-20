#ifndef EKF_INCLUDE_GUARD_HPP
#define EKF_INCLUDE_GUARD_HPP

#include <math.h>
#include <armadillo>
#include "turtlelib/rigid2d.hpp"
/// \file
/// \brief launches turtlebot3 and obstacles in rviz scene for visualization


namespace slam
{

    class ekf
    {
        private:
            double obs_size;
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
            arma::Mat<double> q_previous;
            arma::Mat<double> I;
            arma::Mat<double> z_predict;


        public:

            /// \brief initialize
            explicit ekf();

            /// \brief initialize to the correct size
            /// \param size - set the size of the ekf to start with
            /// \return none
            void ekf_size(int size); 

            /// \brief Set the Q matrix
            /// \param Q - the matrix to set Q to 
            /// \return none
            void setQ(arma::Mat<double> Q);

            /// \brief Set the R matrix
            /// \param R - the matrix to set R to 
            /// \return none
            void setR(arma::Mat<double> R);

            /// \brief Set the A matrix
            /// \param A - the matrix to set A to 
            /// \return none
            void setA(arma::Mat<double> A);

            /// \brief Set the H matrix
            /// \param H - the matrix to set H to 
            /// \return none
            void setH(arma::Mat<double> H);

            /// \brief set initial state and covariance matrices
            /// \param x_0 - Initial state of the robot
            /// \param Sigma_0 - Initial covariance matrix
            /// \return none
            void initial_state(arma::Mat<double> x_0, arma::Mat<double> Sigma_0);

            /// \brief predict the state vector for ekf
            /// \param u - Input twist
            /// \return a vector that predicts the state
            arma::Mat<double> predict_q(turtlelib::Twist u);

            /// \brief calculates A based on state vector for the current time step
            /// \param u - Input twist
            /// \return the calculated A matrix
            arma::Mat<double> calc_A(turtlelib::Twist u);

            /// \brief calculate the H matrix based on the measurement, and the predicted measurement
            /// \param i - the id of the obstacle 
            /// \return the calculated H matrix
            arma::Mat<double> calc_H(int j);

            /// \brief predicts the Covariance matrix
            void predict();

            /// \brief Update stage of EKF algorithm
            /// \param z_measured - the obstacles measurement 
            /// \return the updated state vector
            arma::Mat<double> update(arma::Mat<double> z_measured);

            /// \brief Calculate the measurement prediction for data association 
            /// \param j - the index for data association
            /// \param q_prov - provisional state vector
            /// \return measurement prediction 
            arma::Mat<double> calc_zhat_data_asso(int j, arma::Mat<double> q_prov); 
            
            /// \brief Calculate the H matrix for data association 
            /// \param j - the index for data association
            /// \param q_prov - provisional state vector
            /// \return H matrix 
            arma::Mat<double> calc_H_data_asso(int j, arma::Mat<double> q_prov, int prov_len); 

            /// \brief Data association
            /// \param mu - the landmark measurement
            /// \return the id for the landmark measurement 
            int landmark_association(arma::Mat<double> mu); 

    };

    /// \brief Fit a circle on cluster
    /// \param cluster_group - group of all clusters from laser scan
    /// \return the 2D vector with data for position and radius of the circle
    std::vector<std::vector<double>> circle_fit(std::vector<std::vector<turtlelib::Vector2D>> cluster_gp); 

    /// \brief Classify whether cluster is circle or not
    /// \param cluster - cluster points
    /// \return boolean true or false 
    bool classify_circle(std::vector<turtlelib::Vector2D> cluster); 
}



# endif
