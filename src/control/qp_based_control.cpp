

#include "control/qp_based_control.h"


namespace control{
    namespace qp_control{

        QPControl::QPControl(const std::shared_ptr<robot::AbsRobot>& _robot, Params& _params, double _dt):params(_params), dt(_dt){
            addRobot(_robot);
            nullPosition =  Eigen::VectorXd::Zero(robots[0]->getDof());

            _QP = std::make_shared<control::util::QP>();
            // dsController = std::make_shared<DSController>(3, 5., 5.);
            // dsController = std::make_shared<DSController>(3, params.psvLambda, params.psvDissip* params.psvLambda);

        }
        QPControl::~QPControl(){};
        void QPControl::setInput(){}
        void QPControl::setParams(Params& _params){params = _params;}
        void QPControl::setNullPos(const Eigen::VectorXd& nullPose){nullPosition = nullPose;}

        Eigen::VectorXd QPControl::getOutput() {
            algorithm();
            //todo std:pair 
            return cmdTorque;
        }

        Eigen::VectorXd QPControl::getPlotVariable(){
            return plotVariable;
        }


        void QPControl::setup_matrices_invDyn(const Eigen::VectorXd& desired_acc, const Eigen::VectorXd& weight_0){
            //**[ddot{q}(16)  tau(16)  W(12)]
            size_t dofs = robots[0]->getDof();
            size_t ntaskspace =  3*4;
            size_t _dim =  2*dofs + ntaskspace;
            double maxWrench = +100; // todo
            double minWrench = -100; // todo pass this part to params

            std::vector<Eigen::MatrixXd> A_t;
            std::vector<Eigen::VectorXd> b_t;
            std::vector<Eigen::VectorXd> w_t;
            std::vector<Eigen::MatrixXd> A_c;
            std::vector<Eigen::MatrixXd> b_c;

            w_t.push_back(weight_0);
            Eigen::VectorXd weight_s = Eigen::VectorXd::Constant(2*dofs+ntaskspace,1);
            weight_s.segment(0,dofs) = Eigen::VectorXd::Constant(dofs,0.01);
            weight_s.segment(dofs,dofs) = Eigen::VectorXd::Constant(dofs,1.);
            w_t.push_back(weight_s);

            //** Setup Task
            //Acceleration task
            A_t.push_back(Eigen::MatrixXd::Zero(ntaskspace, _dim));
            b_t.push_back(Eigen::VectorXd::Zero(ntaskspace));

            Eigen::MatrixXd Jacobs = Eigen::MatrixXd::Zero(ntaskspace,dofs);
            Eigen::VectorXd jdot_qdot = Eigen::VectorXd::Zero(ntaskspace);
            for (size_t i = 0; i < 4; i++){
                jdot_qdot.segment(3*i,3) = std::static_pointer_cast<robot::Hand>(robots[0])->getContactHessians(i).transpose();
                Jacobs.block(3*i,4*i,3,4) = std::static_pointer_cast<robot::Hand>(robots[0])->finger[i].jacobian;
            }
            A_t[0].block(0,0,Jacobs.rows(),Jacobs.cols()) = Jacobs;
            b_t[0] = desired_acc - jdot_qdot;
            // stability holder task
            A_t.push_back(Eigen::MatrixXd::Zero(2*dofs + ntaskspace, _dim));
            b_t.push_back(Eigen::VectorXd::Zero(2*dofs + ntaskspace));
            A_t[1].block(0,0,dofs,dofs) = Eigen::MatrixXd::Identity(dofs, dofs);
            A_t[1].block(dofs,dofs,dofs,dofs) = Eigen::MatrixXd::Identity(dofs, dofs);
            A_t[1].block(2*dofs,2*dofs,ntaskspace,ntaskspace) = Eigen::MatrixXd::Identity(ntaskspace, ntaskspace);
            b_t[1].segment(dofs,dofs) = std::static_pointer_cast<robot::Hand>(robots[0])->getGravityComp();

            //**Dynamic Const 
            A_c.push_back(Eigen::MatrixXd::Zero(dofs, _dim));
            b_c.push_back(Eigen::MatrixXd::Zero(2, dofs));
            Eigen::MatrixXd M = std::static_pointer_cast<robot::Hand>(robots[0])->getMassMatrix();
            Eigen::VectorXd Cg = std::static_pointer_cast<robot::Hand>(robots[0])->getCoriolisAndGravityForces();
            Eigen::MatrixXd S = Eigen::MatrixXd::Identity(dofs, dofs);

            A_c[0].block(0, 0, dofs, dofs) = M;
            A_c[0].block(0, dofs, dofs, dofs) = -S;
            A_c[0].block(0, 2*dofs, dofs, ntaskspace) = -Jacobs.transpose();
            b_c[0].row(0) = -Cg.transpose();
            b_c[0].row(1) = -Cg.transpose();

            //*jont velocity limit
            A_c.push_back(Eigen::MatrixXd::Zero(dofs, _dim));
            b_c.push_back(Eigen::MatrixXd::Zero(2, dofs));
            A_c[1].diagonal().head(dofs) = Eigen::VectorXd::Constant(dofs, dt);
            b_c[1].row(0) = -std::static_pointer_cast<robot::Hand>(robots[0])->getVelLimits().transpose();
            b_c[1].row(0) -= robots[0]->getStates().vel.transpose();
            b_c[1].row(1) = std::static_pointer_cast<robot::Hand>(robots[0])->getVelLimits().transpose();
            b_c[1].row(1) -= robots[0]->getStates().vel.transpose();

            //*joint position limit
            A_c.push_back(Eigen::MatrixXd::Zero(dofs, _dim));
            b_c.push_back(Eigen::MatrixXd::Zero(2, dofs));

            Eigen::VectorXd pos_lb, pos_ub;
            std::tie(pos_lb, pos_ub) = std::static_pointer_cast<robot::Hand>(robots[0])->getPosLimits();
            A_c[2].diagonal().head(dofs) = Eigen::VectorXd::Constant(dofs, dt * dt);
            b_c[2].row(0) = pos_lb.transpose();
            b_c[2].row(0) -= (robots[0]->getStates().vel * dt + robots[0]->getStates().pos).transpose();
            b_c[2].row(1) = pos_ub.transpose();
            b_c[2].row(1) -= (robots[0]->getStates().vel * dt + robots[0]->getStates().pos).transpose();

            _QP->setup_qp_tasks(A_t,b_t,w_t);
            _QP->setup_qp_constraints(A_c,b_c);

            // Variable bounds
            Eigen::VectorXd lb = Eigen::VectorXd::Zero(_dim);
            Eigen::VectorXd ub = Eigen::VectorXd::Zero(_dim);
            lb.head(dofs) = (-1/dt)*std::static_pointer_cast<robot::Hand>(robots[0])->getVelLimits();
            ub.head(dofs) = (1/dt)*std::static_pointer_cast<robot::Hand>(robots[0])->getVelLimits();
            lb.segment(dofs, dofs) = -std::static_pointer_cast<robot::Hand>(robots[0])->getTrqLimits();
            ub.segment(dofs, dofs) = std::static_pointer_cast<robot::Hand>(robots[0])->getTrqLimits();
            lb.segment(2*dofs, ntaskspace) = Eigen::VectorXd::Constant(ntaskspace,minWrench);
            ub.segment(2*dofs, ntaskspace) = Eigen::VectorXd::Constant(ntaskspace,maxWrench);

            _QP->setup_qp_bounds(lb,ub);

        }

        void QPControl::setup_matrices_ds(const Eigen::VectorXd& _fx, const Eigen::VectorXd& _tau_t, const Eigen::VectorXd& _tau_g,
         const Eigen::VectorXd& weight_0, const Eigen::VectorXd& weight_1){
            //**[dot{q}(16)  tau(16)]
            size_t dofs = robots[0]->getDof();
            size_t _dim =  2*dofs;
            size_t ntaskspace =  12;

            //** Setup Task
            std::vector<Eigen::MatrixXd> A_t;
            std::vector<Eigen::VectorXd> b_t;
            std::vector<Eigen::VectorXd> w_t;
            std::vector<Eigen::MatrixXd> A_c;
            std::vector<Eigen::MatrixXd> b_c;

            w_t.push_back(weight_0);
            w_t.push_back(weight_1);

            A_t.push_back(Eigen::MatrixXd::Zero(ntaskspace, _dim));
            b_t.push_back(Eigen::VectorXd::Zero(ntaskspace));

            Eigen::MatrixXd Jacobs = Eigen::MatrixXd::Zero(ntaskspace,dofs);
            for (size_t i = 0; i < 4; i++)
                Jacobs.block(3*i,4*i,3,4) = std::static_pointer_cast<robot::Hand>(robots[0])->finger[i].jacobian;
            A_t[0].block(0,0,Jacobs.rows(),Jacobs.cols()) = Jacobs;
            b_t[0] = _fx;

            // stability holder task
            A_t.push_back(Eigen::MatrixXd::Zero(dofs, _dim));
            b_t.push_back(Eigen::VectorXd::Zero(dofs));
            A_t[1].block(0,dofs,dofs,dofs) = Eigen::MatrixXd::Identity(dofs, dofs);


            //**Dynamic Const 
            A_c.push_back(Eigen::MatrixXd::Zero(dofs, _dim));
            b_c.push_back(Eigen::MatrixXd::Zero(2, dofs));

            Eigen::MatrixXd M = std::static_pointer_cast<robot::Hand>(robots[0])->getMassMatrix();
            Eigen::VectorXd Cg = std::static_pointer_cast<robot::Hand>(robots[0])->getCoriolisAndGravityForces();
            Eigen::MatrixXd S = Eigen::MatrixXd::Identity(dofs, dofs);
            double _dt = 0.005;
            A_c[0].block(0, 0, dofs, dofs) = M/_dt;
            A_c[0].block(0, dofs, dofs, dofs) = -S;
            b_c[0].row(0) = ( _tau_t + (1/_dt)*M* robots[0]->getStates().vel).transpose();
            b_c[0].row(1) = ( _tau_t + (1/_dt)*M* robots[0]->getStates().vel).transpose();

            _QP->setup_qp_tasks(A_t,b_t,w_t);
            _QP->setup_qp_constraints(A_c,b_c);
            // Variable bounds
            Eigen::VectorXd lb = Eigen::VectorXd::Zero(_dim);
            Eigen::VectorXd ub = Eigen::VectorXd::Zero(_dim);
            lb.head(dofs) = -std::static_pointer_cast<robot::Hand>(robots[0])->getVelLimits();
            ub.head(dofs) =  std::static_pointer_cast<robot::Hand>(robots[0])->getVelLimits();
            lb.segment(dofs, dofs) =-_tau_t -_tau_g - std::static_pointer_cast<robot::Hand>(robots[0])->getTrqLimits();
            ub.segment(dofs, dofs) =-_tau_t -_tau_g + std::static_pointer_cast<robot::Hand>(robots[0])->getTrqLimits();

            _QP->setup_qp_bounds(lb,ub);

        }

        void QPControl::setup_matrices_qp(const Eigen::VectorXd& _xddot_ref, const double& w_task, const double& w_reg, const double& w_out){
            //**[dot{q}(16)  tau(16)]
            size_t dofs = robots[0]->getDof();
            size_t _dim =  2*dofs;
            size_t ntaskspace =  6*4;

            //** Setup Task
            std::vector<Eigen::MatrixXd> A_t;
            std::vector<Eigen::VectorXd> b_t;
            std::vector<Eigen::VectorXd> w_t;
            std::vector<Eigen::MatrixXd> A_c;
            std::vector<Eigen::MatrixXd> b_c;

            Eigen::MatrixXd Jacobs = Eigen::MatrixXd::Zero(ntaskspace,dofs);
            Eigen::VectorXd Jdot_qdot = Eigen::VectorXd::Zero(ntaskspace);

            for (size_t i = 0; i < 4; i++){
                Jacobs.block(6*i,4*i,6,4) = std::static_pointer_cast<robot::Hand>(robots[0])->getFingerFullJacobian(i);
                Jdot_qdot.segment(6*i,6) = std::static_pointer_cast<robot::Hand>(robots[0])->getContactHessians(i).transpose();
            }

            double _dt = 0.005;
            Eigen::MatrixXd M = std::static_pointer_cast<robot::Hand>(robots[0])->getMassMatrix();
            Eigen::VectorXd Cg = -std::static_pointer_cast<robot::Hand>(robots[0])->getCoriolisAndGravityForces();
            //******************************************************
            //** tracking task
            w_t.push_back(Eigen::VectorXd::Constant(ntaskspace,w_task));
            A_t.push_back(Eigen::MatrixXd::Zero(ntaskspace, _dim));
            b_t.push_back(Eigen::VectorXd::Zero(ntaskspace));
            A_t[0].block(0,0,Jacobs.rows(),Jacobs.cols()) = Jacobs;
            b_t[0] = _xddot_ref - Jdot_qdot;

            //** stability holder task
            w_t.push_back(Eigen::VectorXd::Constant(2*dofs,w_reg));
            w_t[1].tail(dofs) = Eigen::VectorXd::Constant(dofs,w_out);
            A_t.push_back(Eigen::MatrixXd::Identity(_dim, _dim));
            b_t.push_back(Eigen::VectorXd::Zero(2*dofs));
            // b_t[1].tail(dofs) = std::static_pointer_cast<robot::Hand>(robots[0])->getStates().trq;
            b_t[1].tail(dofs) = Cg;


            //**Dynamic Const 
            A_c.push_back(Eigen::MatrixXd::Zero(dofs, _dim));
            b_c.push_back(Eigen::MatrixXd::Zero(2, dofs));
            A_c[0].block(0, 0, dofs, dofs) = M;
            A_c[0].block(0, dofs, dofs, dofs) = -Eigen::MatrixXd::Identity(dofs, dofs);
            b_c[0].row(0) = -Cg;
            b_c[0].row(1) = -Cg;

            //** Variable bounds
            Eigen::VectorXd lb = Eigen::VectorXd::Zero(_dim);
            Eigen::VectorXd ub = Eigen::VectorXd::Zero(_dim);
            lb.head(dofs) = (1/dt)*(-std::static_pointer_cast<robot::Hand>(robots[0])->getVelLimits() - std::static_pointer_cast<robot::Hand>(robots[0])->getStates().vel );
            ub.head(dofs) = (1/dt)*( std::static_pointer_cast<robot::Hand>(robots[0])->getVelLimits() - std::static_pointer_cast<robot::Hand>(robots[0])->getStates().vel );
            lb.segment(dofs, dofs) = - std::static_pointer_cast<robot::Hand>(robots[0])->getTrqLimits();
            ub.segment(dofs, dofs) = std::static_pointer_cast<robot::Hand>(robots[0])->getTrqLimits();

            _QP->setup_qp_tasks(A_t,b_t,w_t);
            _QP->setup_qp_constraints(A_c,b_c);
            _QP->setup_qp_bounds(lb,ub);
        }

        void QPControl::algorithm(){

            size_t dofs = robots[0]->getDof();
            cmdTorque = Eigen::VectorXd::Zero(dofs);
            //********compute the gravity compent
            Eigen::Matrix3d baseRotMat;
            // baseRotMat = Eigen::AngleAxisd(-1*M_PI,Eigen::Vector3d::UnitX());
            // baseRotMat = Eigen::AngleAxisd(-1*M_PI,Eigen::Vector3d::UnitY());
            // std::static_pointer_cast<robot::Hand>(robots[0])->rotateGravity(baseRotMat);

            double ker_sig = 0.004;

            if(_step == 0){
                
                for (double dx = -0.08; dx < 0.071; dx+=0.01)
                    for (double dy = -0.05; dy < 0.051; dy+=0.01)
                        for (double dz = -0.08; dz < 0.081; dz+=0.01){
                            phi.push_back(Eigen::Vector3d(dx,dy,dz));
                        }
                
                for (size_t i = 0; i < 4; i++){
                    Px.push_back(100.*Eigen::MatrixXd::Identity(3,6));
                    Pr.push_back(100.*Eigen::MatrixXd::Identity(3,3));
                    Kx.push_back(Eigen::MatrixXd::Zero(3,3));
                    Kr.push_back(Eigen::Matrix3d::Zero(3,3));
                    Pf.push_back(Eigen::MatrixXd::Zero(3,phi.size()));
                }
            }

            //********set desired task
            //todo move this part in a function
            std::vector<Eigen::Vector3d> targetPositions(4);
            // targetPositions[0] =  Eigen::Vector3d(.05,-.05,.1);
            // targetPositions[1] =  Eigen::Vector3d(.05,  0.,.1);
            // targetPositions[2] =  Eigen::Vector3d(.05, .05,.1);
            // targetPositions[3] =  Eigen::Vector3d(.05,-.05,.05);

            // int c_flag = (int)((_step-100) /300);
            // std::cout <<  c_flag % 5 << std::endl;
            // if (_step < 100 ){
            //     targetPositions[0] =  Eigen::Vector3d(.1,-.07,.15);
            //     targetPositions[1] =  Eigen::Vector3d(.1, -0.015,.15);
            //     targetPositions[2] =  Eigen::Vector3d(.1, .055,.15);
            //     targetPositions[3] =  Eigen::Vector3d(.05, -.05,.05);
            // }else if(c_flag % 5 < 1){
            //     targetPositions[0] =  Eigen::Vector3d(.01,-.05,.17);
            //     targetPositions[1] =  Eigen::Vector3d(.01,  0.,.17);
            //     targetPositions[2] =  Eigen::Vector3d(.01, .05,.17);
            //     targetPositions[3] =  Eigen::Vector3d(.1,  .0,.07);
                
            // }else if(c_flag % 5 < 2){
            //     targetPositions[0] =  Eigen::Vector3d(.07,-.04,.14);
            //     targetPositions[1] =  Eigen::Vector3d(.07,  0.,.14);
            //     targetPositions[2] =  Eigen::Vector3d(.07, .04,.14);
            //     targetPositions[3] =  Eigen::Vector3d(.09, .02,.06);

            // }else if(c_flag % 5 < 3){
            //     targetPositions[0] =  Eigen::Vector3d(.1,-.05,.1);
            //     targetPositions[1] =  Eigen::Vector3d(.1, .015,.1);
            //     targetPositions[2] =  Eigen::Vector3d(.1, .065,.1);
            //     targetPositions[3] =  Eigen::Vector3d(.12, -.05,.02);

            // }else if(c_flag % 5 < 4){
            //     targetPositions[0] =  Eigen::Vector3d(.07,-.05,.17);
            //     targetPositions[1] =  Eigen::Vector3d(.07,-.01,.17);
            //     targetPositions[2] =  Eigen::Vector3d(.07, .04,.17);
            //     targetPositions[3] =  Eigen::Vector3d(.07, .04,.05);

            // }else if(c_flag % 5 < 5){
            //     targetPositions[0] =  Eigen::Vector3d(.08,-.046,.05);
            //     targetPositions[1] =  Eigen::Vector3d(.08, .0  ,.05);
            //     targetPositions[2] =  Eigen::Vector3d(.08, .046,.05);
            //     targetPositions[3] =  Eigen::Vector3d(.095, -.087,.016);

            // }
            std::vector<Eigen::Vector4d> targetJointPositions(4);

            targetJointPositions[0] =  Eigen::Vector4d(-0.0185253, 0.202005,  1.02723, -0.13217);
            targetJointPositions[1] =  Eigen::Vector4d(-0.0594836, 0.353794, 0.602078,  0.36086);
            targetJointPositions[2] =  Eigen::Vector4d(0.00825905, 0.360537, 0.658054,-0.234928);
            targetJointPositions[3] =  Eigen::Vector4d(1.45594,  1.25053, 0.479659,  1.13652);

            //! to test the position traking:
            targetPositions[0] =  Eigen::Vector3d(.07,-.04,.14);
            targetPositions[1] =  Eigen::Vector3d(.07,  0.,.14);
            targetPositions[2] =  Eigen::Vector3d(.07, .04,.14);
            targetPositions[3] =  Eigen::Vector3d(.09, .02,.06);
            //*******get force from passie Ds
            Eigen::VectorXd xddot = Eigen::VectorXd::Constant(4*6, 0.);
            size_t numfingers = std::static_pointer_cast<robot::Hand>(robots[0])->finger.size();
            for (size_t i = 0; i < numfingers ; i++){
                Eigen::Vector3d fing_pos = std::static_pointer_cast<robot::Hand>(robots[0])->finger[i].task_states.pos;
                Eigen::Vector3d fing_vel = std::static_pointer_cast<robot::Hand>(robots[0])->finger[i].task_states.vel;

                if (_step < 10)
                    targetPositions[i] = fing_pos;

                // double kx = control::util::computeRbfGain(fing_pos, targetPositions[i], 2000, 500,0.04);
                // double kv = control::util::computeRbfGain(fing_pos, targetPositions[i], 500, 25,0.04);
                // double kv = 50;
                
                Eigen::VectorXd erPos = fing_pos - targetPositions[i];
                // std::cout << "norm of pos: " << erPos.norm() << std::endl;
                //**only position:
                // Eigen::MatrixXd A_r = -5.*Eigen::Matrix3d::Identity();
                // Eigen::MatrixXd P_l = Eigen::MatrixXd::Identity(3,3);
                // Eigen::MatrixXd B_r = Eigen::MatrixXd::Identity(3,3);
                // Eigen::VectorXd psi = Eigen::VectorXd::Zero(3);

                //**position and velocity:
                Eigen::MatrixXd A_r = Eigen::MatrixXd::Zero(6,6);
                A_r.block(0,3,3,3) = Eigen::Matrix3d::Identity();
                A_r.block(3,0,3,3) = -20.*Eigen::Matrix3d::Identity();
                A_r.block(3,3,3,3) = -10.*Eigen::Matrix3d::Identity();
                Eigen::MatrixXd P_l = Eigen::MatrixXd::Identity(6,6);
                P_l.block(0,0,3,3) = 2*Eigen::Matrix3d::Identity();
                P_l.block(3,3,3,3) = 2*Eigen::Matrix3d::Identity();

                Eigen::MatrixXd B_r = Eigen::MatrixXd::Zero(6,3);
                B_r.block(0,0,3,3) = Eigen::Matrix3d::Identity();

                Eigen::VectorXd psi = Eigen::VectorXd::Zero(6);
                psi.segment(0,3) = fing_pos;
                psi.segment(3,3) = fing_vel;
                Eigen::VectorXd psi_d = Eigen::VectorXd::Zero(6);
                psi_d.segment(0,3) = targetPositions[i];
                Eigen::VectorXd er = psi - psi_d;
                if (er.norm() > 0.04){
                    er = 0.04*er.normalized();
                }else if (er.norm() < 0.01){
                    er = 0.*er;
                }
                if (erPos.norm() > 0.04)
                    erPos = 0.04*erPos.normalized();
                
                Eigen::VectorXd rt= -B_r.transpose() * A_r * psi_d;
                
                double gainConst = 100.;
                double maxValue = 100.;

                Eigen::VectorXd phi_er = Eigen::VectorXd::Zero(phi.size());
                for (size_t j = 0; j < phi_er.size(); j++){
                    double theta = (1/(ker_sig*ker_sig))*(er.segment(0,3) - phi[j]).transpose()*(er.segment(0,3) - phi[j]);
                    phi_er(j) = std::exp(-theta);
                }
                
                Px[i] -= gainConst * B_r.transpose()* P_l * er *  psi.transpose();
                Pr[i] -= gainConst * B_r.transpose()* P_l * er *  rt.transpose();
                // Pf[i] -= 5.* gainConst * B_r.transpose()* P_l * er *  phi_er.transpose();
                Pf[i] -= 5.* gainConst * B_r.transpose()* P_l.block(0,0,6,3) * er.segment(0,3) *  phi_er.transpose();
                
                for (size_t j = 0; j < Px[i].rows(); j++){
                    for (size_t k = 0; k < Px[i].cols(); k++){
                        if(Px[i](j,k) > maxValue){ Px[i](j,k) = maxValue;} 
                        else if(Px[i](j,k) < -maxValue){Px[i](j,k) = -maxValue;}
                    }
                }
                for (size_t j = 0; j < Pr[i].rows(); j++){
                    for (size_t k = 0; k < Pr[i].cols(); k++){
                        if(Pr[i](j,k) > maxValue){ Pr[i](j,k) = maxValue;} 
                        else if(Pr[i](j,k) < -maxValue){Pr[i](j,k) = -maxValue;}
                    }
                }
                for (size_t j = 0; j < Pf[i].rows(); j++){
                    for (size_t k = 0; k < Pf[i].cols(); k++){
                        if(Pf[i](j,k) > maxValue){ Pf[i](j,k) = maxValue;} 
                        else if(Pf[i](j,k) < -maxValue){Pf[i](j,k) = -maxValue;}
                    }
                }

                // xddot.segment(6*i+3,3) = -kx * erPos - kv * fing_vel + Px[i] * psi + Pr[i] * rt;
                // std::cout << "HHHHHHHHHHHHHHHHHHHHHHH 01" << std::endl;
                // xddot.segment(6*i+3,3) = Px[i] * psi + Pr[i] * rt + Pf[i] * phi_er   -kx * erPos - kv * fing_vel;
                // xddot.segment(6*i+3,3) = Pf[i] * phi_er   -1000 * erPos - 100 * fing_vel;

                xddot.segment(6*i+3,3) = Px[i] * psi + Pr[i] * rt + Pf[i] * phi_er;

                
                //** sum up
                if (i==0)
                    plotVariable = fing_pos - targetPositions[i];
            }

            double wt_tas = 100.;
            double wt_reg = .1;
            double wt_out = 1.;

            setup_matrices_qp(xddot,wt_tas, wt_reg, wt_out);
            Eigen::VectorXd sol;
            sol.resize(2*dofs);
            if (_QP->qp_solve())
                sol = _QP->qp_solution();
            cmdTorque =  sol.segment(dofs,dofs);

            if (_step < 100)
                cmdTorque = Eigen::VectorXd::Zero(dofs);

            _step ++;
        }

    }
}

