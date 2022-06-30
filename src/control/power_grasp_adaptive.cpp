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


#include "control/power_grasp_adaptive.h"


namespace control{
    namespace ad_control{

        ADPowerGrasp::ADPowerGrasp(const std::shared_ptr<robot::AbsRobot>& _robot, Params& _params, double _dt):params(_params), dt(_dt){
            addRobot(_robot);
            for (size_t i = 0; i < 4; i++){
                _AD.push_back(std::make_shared<control::util::Adaptive>(8));
            }
            do_grasp = false;
            
        }
        ADPowerGrasp::~ADPowerGrasp(){};
        void ADPowerGrasp::setInput(){};

        void ADPowerGrasp::setCmd(bool cmd){
            do_grasp = cmd;
        }

        void ADPowerGrasp::setParams(Params& _params){params = _params;}

        Eigen::VectorXd ADPowerGrasp::getOutput() {
            algorithm();
            //todo std:pair 
            return cmdTorque;
        }


        //********************************************************
        Eigen::VectorXd ADPowerGrasp::joint_space_control(const size_t& ind,const Eigen::VectorXd& target_pos){
            
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

        //**********************************************************
        void ADPowerGrasp::algorithm(){

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

            std::vector<Eigen::Vector4d> targetJointPositions(4);
            
            // if (_step > 2000)
            //     do_grasp = true;                       
            
            
            if(do_grasp){
                targetJointPositions[0] =  Eigen::Vector4d(0.,  1.3, 1.1, 0.52);
                targetJointPositions[1] =  Eigen::Vector4d(0.,  1.3, 1.1, 0.52);
                targetJointPositions[2] =  Eigen::Vector4d(0.,  1.3, 1.1, 0.52);
                targetJointPositions[3] =  Eigen::Vector4d(1.3, 0.0, 0.3, 1.0);

            }else{
                targetJointPositions[0] =  Eigen::Vector4d(0., 0.3, 0.4, 0.3);
                targetJointPositions[1] =  Eigen::Vector4d(0., 0.3, 0.4, 0.3);
                targetJointPositions[2] =  Eigen::Vector4d(0., 0.3, 0.4, 0.3);
                targetJointPositions[3] =  Eigen::Vector4d(1.4,0.3, 0.2, 0.2);
            }

            
            //*******get force from passie Ds
            for (size_t i = 0; i < numfingers ; i++){                
            
                cmdTorque.segment(4*i,4) = joint_space_control(i,targetJointPositions[i]);
            
                double maxTorque = 0.4;
                for (size_t j = 0; j < 4; j++){
                    if (cmdTorque[4*i+j] > maxTorque ){
                        cmdTorque[4*i+j] = maxTorque;
                    }else if(cmdTorque[4*i+j] < -maxTorque){
                        cmdTorque[4*i+j] = -maxTorque;
                    }
                }
            }
 
            if (_step < 200)
                cmdTorque = Eigen::VectorXd::Zero(dofs);
            

            _step ++;
        }

    }
}

