//|
//|    Copyright (C) 2020 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
//|    Authors:  Farshad Khadivar (maintainer)
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

        std::vector<std::string> ADControl::get_data_header(){
            std::vector<std::string> _header;
            _header.push_back("finger_0_x");_header.push_back("finger_0_y");_header.push_back("finger_0_z");
            _header.push_back("finger_1_x");_header.push_back("finger_1_y");_header.push_back("finger_1_z");
            _header.push_back("finger_2_x");_header.push_back("finger_2_y");_header.push_back("finger_2_z");
            _header.push_back("finger_3_x");_header.push_back("finger_3_y");_header.push_back("finger_3_z");

            _header.push_back("finger_0_xd");_header.push_back("finger_0_yd");_header.push_back("finger_0_zd");
            _header.push_back("finger_1_xd");_header.push_back("finger_1_yd");_header.push_back("finger_1_zd");
            _header.push_back("finger_2_xd");_header.push_back("finger_2_yd");_header.push_back("finger_2_zd");
            _header.push_back("finger_3_xd");_header.push_back("finger_3_yd");_header.push_back("finger_3_zd");

            _header.push_back("finger_0_ex");_header.push_back("finger_0_ey");_header.push_back("finger_0_ez");
            _header.push_back("finger_1_ex");_header.push_back("finger_1_ey");_header.push_back("finger_1_ez");
            _header.push_back("finger_2_ex");_header.push_back("finger_2_ey");_header.push_back("finger_2_ez");
            _header.push_back("finger_3_ex");_header.push_back("finger_3_ey");_header.push_back("finger_3_ez");

            _header.push_back("imp00");_header.push_back("imp01");_header.push_back("imp02");_header.push_back("imp03");
            _header.push_back("imp10");_header.push_back("imp11");_header.push_back("imp12");_header.push_back("imp13");
            _header.push_back("imp20");_header.push_back("imp21");_header.push_back("imp22");_header.push_back("imp23");
            _header.push_back("imp30");_header.push_back("imp31");_header.push_back("imp32");_header.push_back("imp33");

            _header.push_back("trq00");_header.push_back("trq01");_header.push_back("trq02");_header.push_back("trq03");
            _header.push_back("trq10");_header.push_back("trq11");_header.push_back("trq12");_header.push_back("trq13");
            _header.push_back("trq20");_header.push_back("trq21");_header.push_back("trq22");_header.push_back("trq23");
            _header.push_back("trq30");_header.push_back("trq31");_header.push_back("trq32");_header.push_back("trq33");

            return _header;
        }


        Eigen::VectorXd ADControl::getPlotVariable(){
            return plotVariable;
        }
        //
         void ADControl::setup_matrices_qp(const size_t& ind,  const Eigen::VectorXd& _xdot_ref){
            //**[dot{q}(16)  tau(16)]
            
            size_t dofs = robots[0]->getDof()/4;
            size_t ts_size = _xdot_ref.size();
            double dt = 0.005;

            Eigen::VectorXd joint_pos = std::static_pointer_cast<robot::Hand>(robots[0])->finger[ind].joint_states.pos;
            Eigen::VectorXd joint_vel = std::static_pointer_cast<robot::Hand>(robots[0])->finger[ind].joint_states.vel;
            Eigen::MatrixXd Jacob = std::static_pointer_cast<robot::Hand>(robots[0])->getFingerJacob(ind);

            //** Setup Task
            std::vector<Eigen::MatrixXd> A_t;
            std::vector<Eigen::VectorXd> b_t;
            std::vector<Eigen::VectorXd> w_t;
            std::vector<Eigen::MatrixXd> A_c;
            std::vector<Eigen::MatrixXd> b_c;
            w_t.push_back(Eigen::VectorXd::Constant(ts_size,1));
            A_t.push_back(Eigen::MatrixXd::Zero(ts_size, dofs));
            b_t.push_back(Eigen::VectorXd::Zero(ts_size));
            A_t[0] = Jacob;
            b_t[0] = Jacob * joint_pos + dt * _xdot_ref;


            //** tracking task
            w_t.push_back(Eigen::VectorXd::Constant(dofs,0.002));
            A_t.push_back(Eigen::MatrixXd::Identity(dofs, dofs));
            b_t.push_back(Eigen::VectorXd::Zero(dofs));
            A_t[1] = Eigen::MatrixXd::Identity(4,4);
            b_t[1] = Eigen::Vector4d::Constant(4,0);

            //**Equality Const 
            A_c.push_back(Eigen::MatrixXd::Zero(ts_size, dofs));
            b_c.push_back(Eigen::MatrixXd::Zero(2, ts_size));

            //** Variable bounds
            Eigen::VectorXd lb = Eigen::VectorXd::Zero(dofs);
            Eigen::VectorXd ub = Eigen::VectorXd::Zero(dofs);
            Eigen::VectorXd ll;
            Eigen::VectorXd uu;
            std::tie(ll,uu) = std::static_pointer_cast<robot::Hand>(robots[0])->getPosLimits();
            lb = ll.segment(4*ind,4);
            ub = uu.segment(4*ind,4);

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
            // psi.segment(ad_hsize,ad_hsize) = joint_vel;

            Eigen::Vector3d fing_pos = std::static_pointer_cast<robot::Hand>(robots[0])->finger[ind].task_states.pos;

            Eigen::Vector3d pos_d = target_pos;

            Eigen::VectorXd desDs = control::util::computeDsRBF(fing_pos, target_pos,10,100,0.05);
            setup_matrices_qp(ind,desDs);
            Eigen::VectorXd sol;
            sol.resize(4);
            if (_QP->qp_solve())
                sol = _QP->qp_solution();
            Eigen::VectorXd psi_d = Eigen::VectorXd::Zero(ad_ssize);
            psi_d.segment(0,ad_hsize) = sol ;
            psi_d.segment(ad_hsize,ad_hsize) = -5. *(psi.segment(0,ad_hsize) - psi_d.segment(0,ad_hsize)) ;
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

            if(_step == 0){
                Eigen::VectorXd adGains = Eigen::VectorXd::Constant(4,0.35);
                adGains(1) *= 1.;
                adGains(2) *= 1.;
                adGains(3) *= 1.;
                Eigen::Vector4d imp = Eigen::Vector4d(0.15, 0.12, 0.1, 0.05);

                for (size_t i = 0; i < _AD.size(); i++){
                    _AD[i]->setAdaptiveGains(adGains);
                    if(i == 3)
                        imp(3) = 0.15;
                    _AD[i]->setImpedance(imp);
                    _AD[i]->activateImpedance();
                }
                _AD[2]->setAdaptiveGains(0.75*adGains);

            }

            //********set desired task
            //todo move this part in a function
            std::vector<Eigen::Vector3d> targetPositions(4);

            //! to test the position traking:
            targetPositions[0] =  Eigen::Vector3d(.11,-.05,.1);
            targetPositions[1] =  Eigen::Vector3d(.11, .015,.1);
            targetPositions[2] =  Eigen::Vector3d(.11, .065,.1);
            targetPositions[3] =  Eigen::Vector3d(.11, -.04,.05);
            size_t step_jump = 1200;
            if (_step > step_jump ){
                targetPositions[0] =  Eigen::Vector3d(.08,-.07,.15);
                targetPositions[1] =  Eigen::Vector3d(.08, -0.015,.15);
                targetPositions[2] =  Eigen::Vector3d(.08, .055,.15);
                targetPositions[3] =  Eigen::Vector3d(.08, -.05,.07);
            } if (_step > 2*step_jump){
                targetPositions[0] =  Eigen::Vector3d(.1,-.07,.08);
                targetPositions[1] =  Eigen::Vector3d(.1, -0.015,.08);
                targetPositions[2] =  Eigen::Vector3d(.1, .055,.08);
                targetPositions[3] =  Eigen::Vector3d(.1, -.05,.04);

            }if (_step > 3*step_jump){
                targetPositions[0] =  Eigen::Vector3d(.08,-.07,.15);
                targetPositions[1] =  Eigen::Vector3d(.08, -0.015,.15);
                targetPositions[2] =  Eigen::Vector3d(.08, .055,.15);
                targetPositions[3] =  Eigen::Vector3d(.08, -.05,.07);

            } if(_step > 4*step_jump){
                targetPositions[0] =  Eigen::Vector3d(.11,-.05,.1);
                targetPositions[1] =  Eigen::Vector3d(.11, .015,.1);
                targetPositions[2] =  Eigen::Vector3d(.11, .065,.1);
                targetPositions[3] =  Eigen::Vector3d(.11, -.04,.05);
            }

            std::vector<Eigen::Vector4d> targetJointPositions(4);
            targetJointPositions[0] =  Eigen::Vector4d(0., 0.3, 0.7, 0.7);
            targetJointPositions[1] =  Eigen::Vector4d(0., 0.3, 0.7, 0.7);
            targetJointPositions[2] =  Eigen::Vector4d(0., 0.3, 0.7, 0.7);
            targetJointPositions[3] =  Eigen::Vector4d(1.2,1.2, 0.7, 0.7);


            Eigen::VectorXd positions = Eigen::VectorXd::Zero(12);
            Eigen::VectorXd positions_ds = Eigen::VectorXd::Zero(12);
            Eigen::VectorXd positions_er = Eigen::VectorXd::Zero(12);
            Eigen::VectorXd impedances = Eigen::VectorXd::Zero(16);

            //*******get force from passie Ds
            for (size_t i = 0; i < numfingers ; i++){
                Eigen::Vector3d fing_pos = std::static_pointer_cast<robot::Hand>(robots[0])->finger[i].task_states.pos;
                Eigen::Vector3d fing_vel = std::static_pointer_cast<robot::Hand>(robots[0])->finger[i].task_states.vel;

                Eigen::VectorXd erPos = Eigen::VectorXd::Zero(4);
                erPos.segment(0,3) = fing_pos - targetPositions[i];
                
                positions.segment(3*i,3) = fing_pos;
                positions_ds.segment(3*i,3) = targetPositions[i];
                positions_er.segment(3*i,3) = fing_pos - targetPositions[i];
                impedances.segment(4*i,4) = _AD[i]->get_impedance();
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
                    // plotVariable = _AD[i]->get_impedance();
            }
 
            if (_step < 25)
                cmdTorque = Eigen::VectorXd::Zero(dofs);
            
            std::vector<double> _data;
            for (size_t i = 0; i < 12; i++)
                _data.push_back(positions[i]);
            for (size_t i = 0; i < 12; i++)
                _data.push_back(positions_ds[i]);
            for (size_t i = 0; i < 12; i++)
                _data.push_back(positions_er[i]);
            for (size_t i = 0; i < 16; i++)
                _data.push_back(impedances[i]);
            for (size_t i = 0; i < 16; i++)
                _data.push_back(cmdTorque[i]);
            data_log = _data;
            _step ++;
        }

    }
}

