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


#include "control/adaptive_control.h"


namespace control{
    namespace ad_control{

        ADControl::ADControl(const std::shared_ptr<robot::AbsRobot>& _robot, Params& _params, double _dt):params(_params), dt(_dt){
            addRobot(_robot);
            _QP = std::make_shared<control::util::QP>();
            for (size_t i = 0; i < 4; i++){
                _AD.push_back(std::make_shared<control::util::Adaptive>(8));
            }
            
        }
        ADControl::~ADControl(){};
        void ADControl::setInput(){}
        void ADControl::setParams(Params& _params){params = _params;}

        Eigen::VectorXd ADControl::getOutput() {
            algorithm();
            //todo std:pair 
            return cmdTorque;
        }

        Eigen::VectorXd ADControl::getPlotVariable(){
            return plotVariable;
        }
        //
         void ADControl::setup_matrices_qp(const size_t& ind,  const Eigen::VectorXd& _xdot_ref){
            //**[dot{q}(16)  tau(16)]
            
            size_t dofs = robots[0]->getDof()/4;
            size_t ts_size = _xdot_ref.size();

            //** Setup Task
            std::vector<Eigen::MatrixXd> A_t;
            std::vector<Eigen::VectorXd> b_t;
            std::vector<Eigen::VectorXd> w_t;
            std::vector<Eigen::MatrixXd> A_c;
            std::vector<Eigen::MatrixXd> b_c;

            Eigen::MatrixXd Jacob = std::static_pointer_cast<robot::Hand>(robots[0])->getFingerJacob(ind);
            // std::cout << "jacobian:" << std::endl << Jacob << std::endl;
            //******************************************************
            //** tracking task
            w_t.push_back(Eigen::VectorXd::Constant(dofs,1));
            A_t.push_back(Eigen::MatrixXd::Identity(dofs, dofs));
            b_t.push_back(Eigen::VectorXd::Zero(dofs));

            // w_t.push_back(Eigen::VectorXd::Constant(ts_size,10));
            // A_t.push_back(Eigen::MatrixXd::Zero(ts_size, dofs));
            // b_t.push_back(Eigen::VectorXd::Zero(ts_size));
            // A_t[1] = Jacob;
            // b_t[1] = _xdot_ref;
            //**Equality Const 
            A_c.push_back(Eigen::MatrixXd::Zero(ts_size, dofs));
            b_c.push_back(Eigen::MatrixXd::Zero(2, ts_size));

            A_c[0] = Jacob;
            b_c[0].row(0) = _xdot_ref;
            b_c[0].row(1) = _xdot_ref;

            //** Variable bounds
            Eigen::VectorXd lb = Eigen::VectorXd::Zero(dofs);
            Eigen::VectorXd ub = Eigen::VectorXd::Zero(dofs);
            lb = -std::static_pointer_cast<robot::Hand>(robots[0])->getVelLimits().segment(4*ind,4);
            ub =  std::static_pointer_cast<robot::Hand>(robots[0])->getVelLimits().segment(4*ind,4);

            _QP->setup_qp_tasks(A_t,b_t,w_t);
            _QP->setup_qp_constraints(A_c,b_c);
            _QP->setup_qp_bounds(lb,ub);

        }

        //********************************************************
        Eigen::VectorXd ADControl::joint_space_control(const size_t& ind,const Eigen::VectorXd& target_pos){
            
            size_t ad_hsize = target_pos.size();
            size_t ad_ssize = ad_hsize*2;

            Eigen::VectorXd joint_pos = std::static_pointer_cast<robot::Hand>(robots[0])->finger[ind].joint_states.pos;
            Eigen::VectorXd joint_vel = std::static_pointer_cast<robot::Hand>(robots[0])->finger[ind].joint_states.vel;


            Eigen::VectorXd psi = Eigen::VectorXd::Zero(ad_ssize);
            psi.segment(0,ad_hsize) = joint_pos;
            // psi.segment(ad_hsize,ad_hsize) = joint_vel.segment(ii,1);

            Eigen::VectorXd psi_d = Eigen::VectorXd::Zero(ad_ssize);
            psi_d.segment(0,ad_hsize) = target_pos;

            psi_d.segment(ad_hsize,ad_hsize) = -5. *(psi.segment(0,ad_hsize) - psi_d.segment(0,ad_hsize)) ;
            if(psi_d.segment(ad_hsize,ad_hsize).norm() > 1.)
                psi_d.segment(ad_hsize,ad_hsize) = 1. * psi_d.segment(ad_hsize,ad_hsize).normalized();

            _AD[ind]->update(psi,psi_d);
            return _AD[ind]->get_output();
        }

        Eigen::VectorXd ADControl::task_space_control(const size_t& ind,const Eigen::VectorXd& target_pos){

            size_t ad_ssize = 8;
            size_t ad_hsize = ad_ssize/2;

            Eigen::VectorXd joint_pos = std::static_pointer_cast<robot::Hand>(robots[0])->finger[ind].joint_states.pos;
            Eigen::VectorXd joint_vel = std::static_pointer_cast<robot::Hand>(robots[0])->finger[ind].joint_states.vel;


            Eigen::VectorXd psi = Eigen::VectorXd::Zero(ad_ssize);
            psi.segment(0,ad_hsize) = joint_pos;
            // psi.segment(ad_hsize,ad_hsize) = joint_vel.segment(ii,1);

            Eigen::Vector3d fing_pos = std::static_pointer_cast<robot::Hand>(robots[0])->finger[ind].task_states.pos;

            Eigen::Vector3d pos_d = target_pos;
            Eigen::VectorXd psi_d = Eigen::VectorXd::Zero(ad_ssize);
            psi_d.segment(0,ad_hsize) = std::static_pointer_cast<robot::Hand>(robots[0])->getFingerInvKinematic(ind,pos_d);
            psi_d.segment(ad_hsize,ad_hsize) = -5. *(psi.segment(0,ad_hsize) - psi_d.segment(0,ad_hsize)) ;
            // 
            if(psi_d.segment(ad_hsize,ad_hsize).norm() > 1.)
                psi_d.segment(ad_hsize,ad_hsize) = 1. * psi_d.segment(ad_hsize,ad_hsize).normalized();

            _AD[ind]->update(psi,psi_d);
            return _AD[ind]->get_output();
        }

        //**********************************************************
        void ADControl::algorithm(){

            size_t dofs = robots[0]->getDof();
            size_t numfingers = std::static_pointer_cast<robot::Hand>(robots[0])->finger.size();
            cmdTorque = Eigen::VectorXd::Zero(dofs);

            //********compute the gravity compent
            size_t ad_ssize = 8;
            size_t ad_hsize = ad_ssize/2;

            if(_step == 0){
                Eigen::VectorXd adGains = Eigen::VectorXd::Constant(4,0.001);
                adGains(1) *= 2.;
                adGains(2) *= 2.;
                adGains(3) *= 1.;
                Eigen::Vector4d imp = Eigen::Vector4d(0.01, 0.1, 0.1, 0.01);
                for (size_t i = 0; i < _AD.size(); i++){
                    _AD[i]->setAdaptiveGains(adGains);
                    _AD[i]->setImpedance(imp);
                    _AD[i]->activateImpedance();
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
            targetJointPositions[0] =  Eigen::Vector4d(0., 0.3, 0.7, 0.7);
            targetJointPositions[1] =  Eigen::Vector4d(0., 0.3, 0.7, 0.7);
            targetJointPositions[2] =  Eigen::Vector4d(0., 0.3, 0.7, 0.7);
            targetJointPositions[3] =  Eigen::Vector4d(1.2,1.2, 0.7, 0.7);

            //! to test the position traking:
            targetPositions[0] =  Eigen::Vector3d(.1,-.05,.1);
            targetPositions[1] =  Eigen::Vector3d(.1, .015,.1);
            targetPositions[2] =  Eigen::Vector3d(.1, .065,.1);
            targetPositions[3] =  Eigen::Vector3d(.12, -.05,.02);
            
            //*******get force from passie Ds
            for (size_t i = 0; i < numfingers ; i++){
                Eigen::Vector3d fing_pos = std::static_pointer_cast<robot::Hand>(robots[0])->finger[i].task_states.pos;
                Eigen::Vector3d fing_vel = std::static_pointer_cast<robot::Hand>(robots[0])->finger[i].task_states.vel;

                Eigen::VectorXd erPos = Eigen::VectorXd::Zero(4);
                erPos.segment(0,3) = fing_pos - targetPositions[i];

                // cmdTorque.segment(4*i,4) = joint_space_control(i,targetJointPositions[i]);
                cmdTorque.segment(4*i,4) = task_space_control(i,targetPositions[i]);
                double maxTorque = 0.4;
                for (size_t j = 0; j < 4; j++){
                    if (cmdTorque[4*i+j] > maxTorque ){
                        cmdTorque[4*i+j] = maxTorque;
                    }else if(cmdTorque[4*i+j] < -maxTorque){
                        cmdTorque[4*i+j] = -maxTorque;
                    }
                }

                if (i==0)
                    plotVariable = erPos;
            }

 
            // std::cout << "TRQ: " << cmdTorque.transpose()<< std::endl ;
            
            if (_step < 50)
                cmdTorque = Eigen::VectorXd::Zero(dofs);
            _step ++;
        }

    }
}

