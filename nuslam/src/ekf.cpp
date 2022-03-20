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
    void ekf::setQ(arma::Mat<double> Q_set)
    {
        Q = Q_set;
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
            update(1) = (-u.xdot*sin(q_previous(0))/u.thetadot) + (u.xdot*sin((q_previous(0) + u.thetadot))/u.thetadot); 
            update(2) = (u.xdot*cos(q_previous(0))/u.thetadot) - (u.xdot*cos((q_previous(0) + u.thetadot))/u.thetadot);
        }
        
        arma::Mat<double> update_zeros = arma::zeros(2*obs_size, 1); 
        arma::Mat<double> updated = arma::join_cols(update, update_zeros); 
        // updated.print(std::cout << "updated"); 
        q_predict = q_previous + updated;

        return q_predict;
    }

    /// \brief Calculate the A matrix
    arma::Mat<double> ekf::calc_A(turtlelib::Twist u)
    {
        arma::Mat<double> A_q;
        if(u.thetadot==0.0)
        {
            A_q = {{1 ,                           0, 0},
                 {-u.xdot*sin(q_previous(0)),   1, 0},
                 {u.xdot*cos(q_previous(0)),    0, 1}};
        }
        else{
            A_q = {{1 ,                                       0, 0},
                 {(-u.xdot*cos(q_previous(0))/u.thetadot) + (u.xdot*cos((q_previous(0)+u.thetadot))/u.thetadot),    1, 0},
                 {(-u.xdot*sin(q_previous(0))/u.thetadot) + (u.xdot*sin((q_previous(0)+u.thetadot))/u.thetadot),    0, 1}};
        }

        arma::Mat<double> zeros2n_3 = arma::zeros(2*obs_size,3);
        arma::Mat<double> zeros3_2n = arma::zeros(3,2*obs_size);
        arma::Mat<double> zeros2n_2n = arma::eye(2*obs_size, 2*obs_size); 
        A_q = arma::join_rows(A_q, zeros3_2n); 
        arma::Mat<double> zero2n_2n3 = arma::join_rows(zeros2n_3, zeros2n_2n);
        A = arma::join_cols(A_q, zero2n_2n3); 

        // A += I;
        // A.print(std::cout << "A"); 
        
        // ekf::setA(A);
        return A;
    }

    /// \brief Calculate the H matrix
    arma::Mat<double> ekf::calc_H(int j)
    {
        double mx = q_predict(3 + 2*j); 
        double my = q_predict(4 + 2*j); 
        double theta = q_predict(0); 
        double x = q_predict(1);
        double y = q_predict(2);
        
        z_predict(0) = sqrt(pow((mx - x),2)+ pow((my-y),2));
        z_predict(1) = atan2((my-y),(mx-x)) - theta;
        z_predict(1) = turtlelib::normalize_angle(z_predict(1)); 
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

        // Q.print("Q"); 
        // R.print("R");
        // A.print("A");
        // H.print("H");
        // Sigma_previous.print("Sigma_prev");
        // q_previous.print("q_prev");

        return H; 
    }

    /// \brief set initial state and covariance matrices
    void ekf::initial_state(arma::Mat<double> x_0, arma::Mat<double> Sigma_0)
    {
        q_previous = x_0;
        Sigma_previous = Sigma_0;
        // Sigma_previous.print(std::cout<<"Sigma prev"<<std::endl); 
        
    }

    void ekf::predict()
    {
        Sigma_predict = A*Sigma_previous*(A.t()) + Q;
    }

    arma::Mat<double> ekf::update(arma::Mat<double> z_measured)
    {
        // z_measured.print(std::cout << "z_measured" <<std::endl); 
        // H.print(std::cout << "H"); 
        // Sigma_predict.print(std::cout<<"Sigma Pred"<<std::endl); 
        K = (Sigma_predict*(H.t()))*(((H*Sigma_predict*(H.t())) + R).i());
        // K.print(std::cout << "K"<<std::endl); 
        arma::Mat<double> z_diff = z_measured  - z_predict; 
        z_diff(1) = turtlelib::normalize_angle(z_diff(1));

        q_update = q_predict + (K*(z_diff));

        Sigma_update = (I - K*H)*Sigma_predict;

        q_update(0) = turtlelib::normalize_angle(q_update(0)); 

        q_previous = q_update; 
        Sigma_previous = Sigma_update;
        return q_previous; 
    }

    // Data association calculate z expected
    arma::Mat<double> ekf::calc_zhat_data_asso(int j, arma::Mat<double> q_prov)
    {
        double mx = q_prov(3 + 2*j); 
        double my = q_prov(4 + 2*j); 
        double theta = q_prov(0); 
        double x = q_prov(1);
        double y = q_prov(2);

        arma::Mat<double> z_exp(2,1); 
        
        z_exp(0) = sqrt(pow((mx - x),2)+ pow((my-y),2));
        z_exp(1) = atan2((my-y),(mx-x)) - theta;
        z_exp(1) = turtlelib::normalize_angle(z_exp(1));

        return z_exp; 
    }

    // Data association calculate H
    arma::Mat<double> ekf::calc_H_data_asso(int j, arma::Mat<double> q_prov, int prov_len)
    {
        double mx = q_prov(3 + 2*j); 
        double my = q_prov(4 + 2*j); 
        double theta = q_prov(0); 
        double x = q_prov(1);
        double y = q_prov(2);

        // arma::Mat<double> z_exp(2,1); 
        
        // z_exp(0) = sqrt(pow((mx - x),2)+ pow((my-y),2));
        // z_exp(1) = atan2((my-y),(mx-x)) - theta;
        // z_exp(1) = turtlelib::normalize_angle(z_predict(1)); 
        double delx = mx - x; 
        double dely = my - y;

        arma::Mat<double> H_q, H_prov; 
        double d = pow(delx,2) + pow(dely, 2);
        // arma::Mat<double> H_q(3,2); 
        // H_q(0,0) = 0.0;
        // H_q(1,0) = -1.0;
        // H_q(0,1) = -delx/sqrt(d);
        // H_q(1,1) = dely/d;
        // H_q()
        H_q = {{0, -delx/sqrt(d), -dely/sqrt(d)},
               {-1, dely/d, -delx/d}};
        
        arma::Mat<double> H_m = arma::zeros(2,2*prov_len); 
        int index = 2*j;
        H_m(0,index) = delx/sqrt(d);
        H_m(1,index) = -dely/d;
        H_m(0,index+1) = dely/sqrt(d);
        H_m(1,index+1) = delx/d;

        H_prov = arma::join_rows(H_q, H_m); 

        // Q.print("Q"); 
        // R.print("R");
        // A.print("A");
        // H.print("H");
        // Sigma_previous.print("Sigma_prev");
        // q_previous.print("q_prev");

        return H_prov; 
    }

    // Landmark association 
    int ekf::landmark_association(arma::Mat<double> mu)
    {
        double r = mu(0); 
        double phi = mu(1); 
        double theta = q_previous(0); 
        double x = q_previous(1);
        double y = q_previous(2);

        // double mx = x + r*cos(phi+theta); 
        // double my = y + r*sin(phi+theta); 

        double mx = r*cos(phi); 
        double my = r*sin(phi); 

        arma::Mat<double> m(2,1); 
        m(0) = mx; 
        m(1) = my; 

        arma::Mat<double> q_prov, cov_prov; 

        q_prov = arma::join_cols(q_previous,m); 


        double obs_size_prov = obs_size + 1;
        std::vector<double> distances; 

        int new_size = 3+2*obs_size_prov; 

        int id = 0; 
        double min_thres = 0.0005;
        bool flag_done = false; 
        for(int i=0; i<obs_size_prov-1; i++)
        {
            arma::Mat<double> H_asso, Sig_prov; 
            H_asso = ekf::calc_H_data_asso(i,q_prov,obs_size_prov); 
            arma::Mat<double> z_hat; 
            z_hat = ekf::calc_zhat_data_asso(i,q_prov); 

            arma::Mat<double> z_diff = mu - z_hat; 
            Sig_prov = resize(Sigma_previous, new_size, new_size); 
            Sig_prov(new_size-2,new_size-2) = 100000; 
            Sig_prov(new_size-1,new_size-1) = 100000; 
            cov_prov = H_asso*Sig_prov*(H_asso.t()) + R;
            arma::Mat<double> z_vec(2,1); 
            z_vec(0) = z_diff(0); 
            z_vec(1) = z_diff(1);
               
            auto dist = z_vec.t()*cov_prov.i()*(z_vec); 
            double d; 
            id = i; 
            
            // if(d < min_thres)
            // {
            //     flag_done = true; 
            //     break; 
            // }

            d = dist.eval()(0,0);  
            // std::cout << " distance: " << d << std::endl;
            distances.push_back(d); 

        } 
        
        // if(flag_done)
        // {
        //     std::cout << "flag_done. ID: " << id << std::endl; 
        // }

        if(distances.size() > 0)
        {
        int minElementIndex = std::min_element(distances.begin(),distances.end()) - distances.begin();
        double minElement = *std::min_element(distances.begin(), distances.end());

        std::cout << "Min element index" << minElementIndex << std::endl; 
        // std::cout << "Min element" << minElement << std::endl;

        if(minElementIndex < obs_size && minElement < 4)
        {
            return minElementIndex; 
        }
        else{

            // arma::Mat<double> Sigma_set, A_set, H_set, Q_set, q_set;            
            // q_set = q_prov;
            // A_set = resize(A,new_size,new_size); 
            // Sigma_set = resize(Sigma_previous, new_size, new_size);
            // H_set = resize(H, 2, new_size);
            // Q_set = resize(Q,new_size,new_size); 
            // Q_set(new_size,new_size) = 0.01; 
            // Q_set(new_size,new_size) = 0.01;
            
            // ekf::setQ(Q_set); 
            // ekf::setA(A_set); 
            // ekf::setH(H_set); 
            // Sigma_previous = Sigma_set; 
            // obs_size = obs_size_prov; 
            
            // return obs_size_prov - 1; 
            return minElementIndex;

        }

        }

    }

    // circle fitting on cluster
    std::vector<std::vector<double>> circle_fit(std::vector<std::vector<turtlelib::Vector2D>> cluster_gp)
    {
        int group_len = int(cluster_gp.size()); 

        std::vector<std::vector<double>> circle_data; 

        // std::cout << "group len" << group_len << std::endl; 

        
        for(int i=0; i<group_len; i++)
        {
            // create variable for cluster
            std::vector<turtlelib::Vector2D> cluster; 
            cluster = cluster_gp.at(i); 

            bool flag = false; 
            // std::cout << "flag prev " << flag << std::endl;             if(flag)
            
            flag = slam::classify_circle(cluster); 
            // std::cout << "flag " << flag << std::endl;             
            if(flag)
            {           
            // variables for centroid
            double c_x = 0.0; double c_y = 0.0; 

            int len = cluster.size(); 

            // Create vectors for x, y 
            // std::cout << "cluster len " << len << std::endl; 
            arma::Mat<double> X_arr(len,1), Y_arr(len,1), Z_arr(len,1); 
            
            for(int j=0; j<len; j++)
            {
                turtlelib::Vector2D p; 
                p = cluster.at(j); 

                X_arr(j) = p.x; 
                Y_arr(j) = p.y; 

                c_x += p.x; 
                c_y += p.y; 
            }
            
            // X_arr.print("X");
            // Y_arr.print("Y"); 
            // ROS_INFO_STREAM("X_arr before" << X_arr);
            // ROS_INFO_STREAM("Y_arr before" << Y_arr); 
            c_x = c_x/len; 
            c_y = c_y/len; 

            X_arr -= c_x; 
            Y_arr -= c_y; 

            // X_arr.print("X");
            // Y_arr.print("Y"); 

            // ROS_INFO_STREAM("X_arr after" <<  X_arr); 
            // ROS_INFO_STREAM("Y_arr after" << Y_arr);
            double z_mean = 0.0; 
            for(int j=0; j<len; j++)
            {
                Z_arr(j) = pow(X_arr(j),2) + pow(Y_arr(j),2); 
                z_mean += Z_arr(j); 
            } 
            z_mean = z_mean/len;

            arma::Mat<double> Z = arma::join_rows(Z_arr,X_arr); 
            
            Z = arma::join_rows(Z,Y_arr); 

            arma::Mat<double> vec_ones = arma::ones(len,1); 

            Z = arma::join_rows(Z,vec_ones); 
            
            // Z.print("Z");
            // ROS_INFO_STREAM("Z after" <<  Z); 

            arma::Mat<double> M = (Z.t())*(Z)/len;

            // M.print("M"); 

            arma::Mat<double> H;
            H = {{8*z_mean, 0, 0, 2},
                {0,        1, 0, 0},
                {0,        0, 1, 0},
                {2,        0, 0, 0},
                }; 
            arma::Mat<double> H_inv; 

            H_inv = {{0, 0, 0, 0.5},
                {0,        1, 0, 0},
                {0,        0, 1, 0},
                {0.5,        0, 0, -2*z_mean},
                }; 
            
            // H_inv = arma::inv(H); 
            // ROS_INFO_STREAM("H_inv" <<  H_inv);
            arma::Mat<double> U, V;
            arma::vec s, s_q; 
            arma::svd(U,s,V,Z); 

            // s.print("s");
            double eig4 = s(3); 
            // ROS_INFO_STREAM("V after" <<  V);
            arma::Mat<double> A(4,1), Y_mat, Q_mat; 
            // std::cout << " eig " << eig4 <<std::endl;
            if(eig4<10e-12)
            {
                A(0) = V(0,3); 
                A(1) = V(1,3);
                A(2) = V(2,3);
                A(3) = V(3,3);
            }
            else{
                arma::Mat<double> Sig, A_st(4,1), Eigvec; 
                Sig = diagmat(s); 
                // Sig.print("Sig"); 
                // ROS_INFO_STREAM("Sigma" <<  Sig);
                Y_mat = V*Sig*(V.t()); 
                Q_mat = Y_mat*H_inv*Y_mat; 
                // ROS_INFO_STREAM("Q" <<  Q_mat);
                // ROS_INFO_STREAM("Y_mat" <<  Y_mat);
                
                arma::eig_sym(s_q,Eigvec,Q_mat); 
                // s_q.print("s_q"); 
                int min_sig_index = 3;
                // ROS_INFO_STREAM("s_q" <<  s_q);
                for(int i=0; i<4; i++)
                {
                    if(s_q(i)>0)
                    {
                        min_sig_index = i;
                        break;
                    } 
                }
                // ROS_INFO_STREAM("min_sig_index" <<  min_sig_index);
                // std::cout << "min sigma index" << min_sig_index << std::endl; 
                A_st(0) = Eigvec(0,min_sig_index); 
                A_st(1) = Eigvec(1,min_sig_index); 
                A_st(2) = Eigvec(2,min_sig_index); 
                A_st(3) = Eigvec(3,min_sig_index);
                A = inv(Y_mat)*A_st; 

            }
            // ROS_INFO_STREAM("A" <<  A);

            double a = -A(1)/(2*A(0));  
            double b = -A(2)/(2*A(0)); 
            double R_s = (pow(A(1),2)+pow(A(2),2)-(4*A(0)*A(3)))/(4*pow(A(0),2)); 
            a += c_x; 
            b += c_y;
            // std::cout << " a " << a <<std::endl;
            // std::cout << " b " << b <<std::endl;
            // std::cout << " R_s " << R_s <<std::endl;
            std::vector<double> data = {a,b,R_s}; 
            // ROS_INFO_STREAM("data a" <<  a);
            // ROS_INFO_STREAM("data b" <<  b);
            // ROS_INFO_STREAM("data R" <<  sqrt(R_s));
            
            if(sqrt(R_s) < 0.05)
            {
            circle_data.push_back(data); 
            }
                

        }

        }
        return circle_data; 
    
    }

    // classify whether a circle or not
    bool classify_circle(std::vector<turtlelib::Vector2D> cluster)
    {

        turtlelib::Vector2D p_first, p_last, p, p1, p2;
        // std::cout << "classify in " << std::endl; 
        p_first = cluster[0]; 
        p_last = cluster[cluster.size()-1]; 
        

        int len_cluster = int(cluster.size()); 
        
        std::vector<double> angles; 

        double angle_mean = 0.0; 

        // std::cout << "len_cluster " << len_cluster << std::endl; 
        for (int i = 1; i<len_cluster-1; i++)
        {
            p = cluster.at(i); 
            // std::cout << "p.x " << p.x <<std::endl;
            // std::cout << "p.y " << p.y <<std::endl;
            p1 = p_first - p; 
            p2 = p - p_last; 

            // std::cout << "p1.x " << p1.x <<std::endl;
            // std::cout << "p1.y " << p1.y <<std::endl;
            // std::cout << "p2.x " << p2.x <<std::endl;
            // std::cout << "p2.y " << p2.y <<std::endl;
            double ang = p1.angle(p1, p2); 
            // std::cout << "angle" << ang <<std::endl;
            ang = turtlelib::rad2deg(ang); 
            angles.push_back(ang); 
            angle_mean += ang; 
            // std::cout << "angle_mean" << angle_mean <<std::endl;

        }
        angle_mean = angle_mean/angles.size(); 

        double stddev = 0.0; 
        for(int i=0; i<angles.size(); i++){
            stddev += pow(turtlelib::deg2rad(angles[i]-angle_mean), 2); 
        }

        stddev = sqrt(stddev/angles.size()); 

        bool flag = false; 
        if (stddev<0.15 && angle_mean>60 && angle_mean<150 )
        { 
    
                flag = true; 
        }

        if(isnan(angle_mean))
        {
            flag = true; 
        }

        return flag; 
    }

}



