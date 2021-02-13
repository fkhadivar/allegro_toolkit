//|
//|    Copyright (C) 2020 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
//|    Authors:  Farshad Khadivr (maintainer)
//|    email:   farshad.khadivar@epfl.ch
//|    website: lasa.epfl.ch
//|
//|    This file is part of allegro_toolkit.
//|
//|    allegro_toolkit is free software: you can redistribute it and/or modify
//|    it under the terms of the GNU General Public License as published by
//|    the Free Software Foundation, either version 3 of the License, or
//|    (at your option) any later version.
//|
//|    allegro_toolkit is distributed in the hope that it will be useful,
//|    but WITHOUT ANY WARRANTY; without even the implied warranty of
//|    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//|    GNU General Public License for more details.
//|

#include "control/control_util.h"
namespace control{
    namespace util{

        QP::QP(){
            _solver = std::make_shared<control::qp_solver::QPOases>();
        }
        QP::~QP(){};
        void QP::setup_qp_tasks(std::vector<Eigen::MatrixXd>& A_t, std::vector<Eigen::VectorXd>& b_t, std::vector<Eigen::VectorXd>& w_t){
            size_t _dim = A_t[0].cols(); 
            size_t num_of_tasks =0;
            for (size_t i = 0; i < A_t.size(); i++)
                num_of_tasks += A_t[i].rows();
            Eigen::MatrixXd A = Eigen::MatrixXd::Zero(num_of_tasks , _dim);
            Eigen::VectorXd b = Eigen::VectorXd::Zero(num_of_tasks);
            size_t t_index = 0;
            for (int i = 0; i < A_t.size(); i++){
                for (int j = 0; j < A_t[i].rows(); j++)
                    A_t[i].row(j).array() *= w_t[i](j);
                b_t[i].array() *= w_t[i].array();
                A.block(t_index, 0, A_t[i].rows(), A_t[i].cols()) = A_t[i];
                b.segment(t_index, b_t[i].size()) = b_t[i].transpose();
                t_index +=A_t[i].rows();
            }
            _H = A.transpose() * A;
            _g = -A.transpose() * b;
        }
        void QP::setup_qp_constraints(std::vector<Eigen::MatrixXd>& A_c, std::vector<Eigen::MatrixXd>& b_c){
            size_t _dim = A_c[0].cols(); 
            size_t num_of_constraints =0;
            for (size_t i = 0; i < A_c.size(); i++)
                num_of_constraints += A_c[i].rows();
            _A = Eigen::MatrixXd::Zero(num_of_constraints, _dim);
            _ubA = Eigen::VectorXd::Zero(num_of_constraints);
            _lbA = Eigen::VectorXd::Zero(num_of_constraints);
            size_t c_index = 0;
            for (int i = 0; i < A_c.size(); i++){
                _A.block(c_index, 0,  A_c[i].rows(),   A_c[i].cols()) = A_c[i];
                _lbA.segment(c_index, b_c[i].cols()) = b_c[i].row(0);
                _ubA.segment(c_index, b_c[i].cols()) = b_c[i].row(1);
                c_index +=A_c[i].rows();
            }
        }
        void QP::setup_qp_bounds(Eigen::VectorXd& lb, Eigen::VectorXd& ub){
            _lb = lb;
            _ub = ub;
        }

        bool QP::qp_solve(){
            if (_solver != nullptr)
                    return _solver->solve(_H, _g, _A, _lb, _ub, _lbA, _ubA);
            return false;
        }

        Eigen::VectorXd QP::qp_solution() const{
            if (_solver)
                return _solver->get_solution();
            return Eigen::VectorXd();
        }
        //**=====================================================================
        //**=====================================================================
        PassiveDS::PassiveDS(const double& lam0, const double& lam1):eigVal0(lam0),eigVal1(lam1){
            set_damping_eigval(lam0,lam1);
        }

        PassiveDS::~PassiveDS(){}
        void PassiveDS::set_damping_eigval(const double& lam0, const double& lam1){
            if((lam0 > 0)&&(lam1 > 0)){
                eigVal0 = lam0;
                eigVal1 = lam1;
                damping_eigval(0,0) = eigVal0;
                damping_eigval(1,1) = eigVal1;
                damping_eigval(2,2) = eigVal1;
            }else{
                std::cerr << "wrong values for the eigenvalues"<<"\n";
            }
        }
        void PassiveDS::updateDampingMatrix(const Eigen::Vector3d& ref_vel){ 

            if(ref_vel.norm() > 1e-6){
                baseMat.setRandom();
                baseMat.col(0) = ref_vel.normalized();
                for(uint i=1;i<3;i++){
                    for(uint j=0;j<i;j++)
                        baseMat.col(i) -= baseMat.col(j).dot(baseMat.col(i))*baseMat.col(j);
                    baseMat.col(i).normalize();
                }
                Dmat = baseMat*damping_eigval*baseMat.transpose();
            }else{
                Dmat = Eigen::Matrix3d::Identity();
            }
            // otherwise just use the last computed basis
        }

        void PassiveDS::update(const Eigen::Vector3d& vel, const Eigen::Vector3d& des_vel){
            // compute damping
            updateDampingMatrix(des_vel);
            // dissipate
            control_output = - Dmat * vel;
            // compute control
            control_output += eigVal0*des_vel;
        }
        Eigen::Vector3d PassiveDS::get_output(){ return control_output;}
        
        //**=====================================================================
        //**=====================================================================
        Adaptive::Adaptive(const size_t& state_size){

            size_t ad_hsize = state_size/2;

            output = Eigen::VectorXd::Zero(ad_hsize);

            Px = Eigen::MatrixXd::Zero(ad_hsize,state_size);
            Pr = Eigen::MatrixXd::Zero(ad_hsize,ad_hsize);
            A_r = Eigen::MatrixXd::Zero(state_size,state_size);

            A_r.block(0,ad_hsize,ad_hsize,ad_hsize) = Eigen::MatrixXd::Identity(ad_hsize,ad_hsize);
            A_r.block(ad_hsize,0,ad_hsize,ad_hsize) = -2.*Eigen::MatrixXd::Identity(ad_hsize,ad_hsize);
            A_r.block(ad_hsize,ad_hsize,ad_hsize,ad_hsize) = -1.*Eigen::MatrixXd::Identity(ad_hsize,ad_hsize);
            
            B_r = Eigen::MatrixXd::Zero(state_size,ad_hsize);
            B_r.block(0,0,ad_hsize,ad_hsize) = Eigen::MatrixXd::Identity(ad_hsize,ad_hsize);

            P_l = Eigen::MatrixXd::Identity(state_size,state_size);
            P_l.block(0,0,ad_hsize,ad_hsize) = 2*Eigen::MatrixXd::Identity(ad_hsize,ad_hsize);
            P_l.block(ad_hsize,ad_hsize,ad_hsize,ad_hsize) = 2*Eigen::MatrixXd::Identity(ad_hsize,ad_hsize);
                        
            Gamma = 1.e-3 * Eigen::MatrixXd::Identity(ad_hsize,ad_hsize);
            Imp = Eigen::VectorXd::Constant(ad_hsize,0.1);
        }
        
        Adaptive::~Adaptive(){}
        void Adaptive::update(const Eigen::VectorXd& c_state, const Eigen::VectorXd& d_state){
            double dt = 0.005;
            Eigen::VectorXd rt= -B_r.transpose() * A_r * d_state;
            Eigen::VectorXd er = c_state - d_state;
            if (er.norm() > error_tol){
                er = error_tol*er.normalized();
            }else if (er.norm() < 0.1*error_tol){
                er = 0.*er;
            }

            size_t ad_hsize = c_state.size()/2;

            Eigen::MatrixXd ThetaX = Eigen::MatrixXd::Zero(ad_hsize,ad_hsize);
            Eigen::MatrixXd ThetaR = Eigen::MatrixXd::Zero(ad_hsize,ad_hsize);
           
            for (size_t j = 0; j < ad_hsize; j++){   
                double ker_sig = 0.02;
                double alp_e = 0.1*std::exp((-0.5/(ker_sig*ker_sig))*er(j)*er(j));
                
                double alp_eX = alp_e;
                double alp_eR = alp_e;
                double normX = Px.row(j).norm();
                double normR = Pr.row(j).norm(); 
                if (normX < 1e-3){
                    normX = 1.;
                    alp_eX = 0;
                }
                if(normR < 1e-3){
                    normR = 1.;
                    alp_eR = 0;
                }
                ThetaX(j,j) = alp_eX * (1. - Imp(j)/normX);
                ThetaR(j,j) = alp_eR * (1. - Imp(j)/normR);
            }

            double imp_act = 0;
            if(is_impedance_active)
                imp_act = 1.;
            auto Px_ = Px;
            auto Pr_ = Pr;
            Px -= (Gamma * B_r.transpose()* P_l * er *  c_state.transpose() + imp_act * ThetaX * Px_)*dt;                     
            Pr -= (Gamma * B_r.transpose()* P_l * er *  rt.transpose()  + imp_act * ThetaR * Pr_)*dt;

            for (size_t j = 0; j < Px.rows(); j++){
                for (size_t k = 0; k < Px.cols(); k++){
                    if(Px(j,k) > max_value){ Px(j,k) = max_value;} 
                    else if(Px(j,k) < -max_value){Px(j,k) = -max_value;}
                }
            }
            for (size_t j = 0; j < Pr.rows(); j++){
                for (size_t k = 0; k < Pr.cols(); k++){
                    if(Pr(j,k) > max_value){ Pr(j,k) = max_value;} 
                    else if(Pr(j,k) < -max_value){Pr(j,k) = -max_value;}
                }
            }

            output =  Px * c_state + Pr * rt ;
        }
        Eigen::VectorXd Adaptive::get_impedance(){
            Eigen::VectorXd imp;
            imp.resize(Px.rows());
            for (size_t i = 0; i < imp.size(); i++){
                imp[i] =  0.5*(Px.row(i).norm() +  Pr.row(i).norm());
            }
            return imp;
        }
    
        

        //**=====================================================================
        //**=====================================================================
        Eigen::VectorXd computeDs(const Eigen::VectorXd& pos, const Eigen::VectorXd& desPos,
                                        const double& dsGain, const double& maxDx){
                Eigen::VectorXd deltaX = desPos - pos;
                if (deltaX.norm() > maxDx)
                    deltaX = maxDx * deltaX.normalized();
                Eigen::VectorXd desVel = dsGain * deltaX;
                return desVel;
        }

        Eigen::VectorXd computeDsRBF(const Eigen::VectorXd& pos, const Eigen::VectorXd& desPos,
                                        const double& dsGain, const double& dsRbfGain, const double& maxDx, const double& lambda){
                Eigen::VectorXd deltaX = desPos - pos;
                if (deltaX.norm() > maxDx)
                    deltaX = maxDx * deltaX.normalized();
                
                Eigen::VectorXd rbf_vector =  Eigen::VectorXd::Zero(pos.size());
                rbf_vector.head(pos.size()) =  deltaX;
                Eigen::MatrixXd Sigma = (1/(4*maxDx*maxDx)) * Eigen::MatrixXd::Identity(3,3);
                double thetagain = -0.5 *  rbf_vector.transpose() * Sigma * rbf_vector;
                Eigen::VectorXd desVel = (1/lambda)*(dsGain + dsRbfGain * std::exp(thetagain)) * deltaX;
                return desVel;
        }
        double computeRbfGain(const Eigen::VectorXd& x, const Eigen::VectorXd& xd,
                                        const double& maxGain, const double& minGain, const double& maxDx){
                Eigen::VectorXd deltaX = xd - x;
                if (deltaX.norm() > maxDx)
                    deltaX = maxDx * deltaX.normalized();
                
                Eigen::VectorXd rbf_vector =  Eigen::VectorXd::Zero(x.size());
                rbf_vector.head(x.size()) =  deltaX;
                Eigen::MatrixXd Sigma = (1/(4*maxDx*maxDx)) * Eigen::MatrixXd::Identity(3,3);
                double thetagain = -0.5 *  rbf_vector.transpose() * Sigma * rbf_vector;
                return minGain + (maxGain-minGain) * std::exp(thetagain);
        }

        Eigen::VectorXd getNullTorque(const size_t& ind, const double& nullGain, const Eigen::Vector4d& nullpose,
                                    const Eigen::Vector4d& robpose, const Eigen::MatrixXd& fing_jacob){

            Eigen::VectorXd temNullTorque = Eigen::VectorXd::Zero(nullpose.size());
            for (size_t j = 0; j < nullpose.size(); j++){
                temNullTorque[j] = -.05 * (robpose[j]  - nullpose[j]);
            }
            Eigen::MatrixXd tempMat = fing_jacob * fing_jacob.transpose();
            Eigen::Matrix4d nullMat =  Eigen::MatrixXd::Identity(nullpose.size(),nullpose.size()) - fing_jacob.transpose() * tempMat.inverse() * fing_jacob;
            Eigen::Vector4d nullTorque = nullMat * temNullTorque;
            if (nullTorque[0] != nullTorque[0])
                nullTorque = Eigen::VectorXd::Zero(nullpose.size());
            return nullTorque;
        }

        double smoothVel(const Eigen::Vector3d& vel, const double& minGain,const double& sigma){
            double _gain = 1.;
            double Sig = 1. / (sigma*sigma);
            _gain = std::exp(-0.5 *Sig* vel.transpose() * vel);
            if(_gain < minGain)
                _gain = minGain;
            return _gain;
        }
    } // namespace util
} // namespace control
