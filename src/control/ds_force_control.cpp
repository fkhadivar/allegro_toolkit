
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

#include "control/ds_force_control.h"
#include <memory>



namespace control {
    namespace ds_force{


        DsForceControl::DsForceControl(const std::shared_ptr<robot::AbsRobot>& _robot, Params& _params):params(_params){
            addRobot(_robot);
            dsController = std::make_shared<control::util::PassiveDS>(params.psvLambda, params.psvDissip*params.psvLambda);
            nullPosition =  Eigen::VectorXd::Zero(robots[0]->getDof()); // todo move this to the hand
        }
        void DsForceControl::setInput(){
        }
        void DsForceControl::setParams(Params& _params){
            params = _params;
            dsController->set_damping_eigval(params.psvLambda, params.psvDissip*params.psvLambda);
        }
        void DsForceControl::setDsGain(const double dsGain){
            params.dsGain = dsGain;
        }
        void DsForceControl::setPsvGain(const double psvLambda, const double psvDissip){
            params.psvLambda = psvLambda;
            params.psvDissip = psvDissip;
            dsController->set_damping_eigval(params.psvLambda, params.psvDissip*params.psvLambda);
        }
        void DsForceControl::setNullPos(const Eigen::VectorXd& nullPose){
            nullPosition = nullPose;
        }

        Eigen::VectorXd DsForceControl::getOutput() {
            algorithm();
            //todo std:pair 
            return cmdTorque;
        }

        std::vector<std::string> DsForceControl::get_data_header(){
            std::vector<std::string> _header;
            _header.push_back("finger_0_x");_header.push_back("finger_0_y");_header.push_back("finger_0_z");
            _header.push_back("finger_1_x");_header.push_back("finger_1_y");_header.push_back("finger_1_z");
            _header.push_back("finger_2_x");_header.push_back("finger_2_y");_header.push_back("finger_2_z");
            _header.push_back("finger_3_x");_header.push_back("finger_3_y");_header.push_back("finger_3_z");

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

        Eigen::VectorXd DsForceControl::getPlotVariable(){
            return plotVariable;
        }

        void DsForceControl::updateHandBasePose(const Eigen::Matrix3d& ref2Base){
            /*
                        left hand:    #  #  #
                ref: ^                   #  #  #
                    z|                   #  #  # 
                    |              ############
                x @-------->y         #######    
            */
        //    _Ref2Base = ref2Base;
        //    _gravity  = _Ref2Base * _gravity;
        //    _baseQuat = Utils<double>::rotationMatrixToQuaternion(_Ref2Base);

        }


        void DsForceControl::algorithm(){
            cmdTorque = Eigen::VectorXd::Zero(robots[0]->getDof());

            //this params are those to get the most out of psv ds
            // controlParams.dsGain = 1.;
            // controlParams.dsMaxVel = .05;
            // controlParams.dsRbfGain = 15.;
            // controlParams.psvLambda = 5.;
            // controlParams.psvDissip = .5;
            // controlParams.nullGain = 1.;
            //********set desired task
            //todo move this part in a function
            std::vector<Eigen::Vector3d> targetPositions(4);
            // targetPositions[0] =  Eigen::Vector3d(.05,-.05,.17);
            // targetPositions[1] =  Eigen::Vector3d(.05,  0.,.17);
            // targetPositions[2] =  Eigen::Vector3d(.05, .05,.17);
            // targetPositions[3] =  Eigen::Vector3d(.05,-.05,.05);

            // if (_step > 2000 )
            // {
            //     targetPositions[0] =  Eigen::Vector3d(.1,-.05,.08);
            //     targetPositions[1] =  Eigen::Vector3d(.1,  0.,.08);
            //     targetPositions[2] =  Eigen::Vector3d(.1, .05,.08);
            //     targetPositions[3] =  Eigen::Vector3d(.05, .0,.0);
            // }

            targetPositions[0] =  Eigen::Vector3d(.11,-.05,.1);
            targetPositions[1] =  Eigen::Vector3d(.11, .015,.1);
            targetPositions[2] =  Eigen::Vector3d(.11, .065,.1);
            targetPositions[3] =  Eigen::Vector3d(.11, -.04,.05);

            if (_step > 2000 ){
                targetPositions[0] =  Eigen::Vector3d(.08,-.07,.15);
                targetPositions[1] =  Eigen::Vector3d(.08, -0.015,.15);
                targetPositions[2] =  Eigen::Vector3d(.08, .055,.15);
                targetPositions[3] =  Eigen::Vector3d(.08, -.05,.07);
            }
            
            // if (_step > 1000)
            // {
            //     targetPositions[0] =  Eigen::Vector3d(.1,-.07,.15);
            //     targetPositions[1] =  Eigen::Vector3d(.1,  0.,.15);
            //     targetPositions[2] =  Eigen::Vector3d(.1, .07,.15);
            //     targetPositions[3] =  Eigen::Vector3d(.05, -.05,.05);
            // }
            // if (_step > 1500)
            // {
            //     targetPositions[0] =  Eigen::Vector3d(.1,-.05,.08);
            //     targetPositions[1] =  Eigen::Vector3d(.1,  0.,.08);
            //     targetPositions[2] =  Eigen::Vector3d(.1, .05,.08);
            //     targetPositions[3] =  Eigen::Vector3d(.05, .0,.0);
            // }

            //*******get force from passie Ds
            Eigen::VectorXd positions = Eigen::VectorXd::Zero(12);
            Eigen::VectorXd impedances = Eigen::VectorXd::Zero(16);
            size_t numfingers = std::static_pointer_cast<robot::Hand>(robots[0])->finger.size();
            for (size_t i = 0; i < numfingers ; i++){
                Eigen::Vector3d fing_pos = std::static_pointer_cast<robot::Hand>(robots[0])->getFingerPos(i);
                //** get the Ds
                if (_step < 10){
                    targetPositions[i] = fing_pos;
                }
                positions.segment(3*i,3) =  fing_pos - targetPositions[i];
                Eigen::Vector3d fing_vel = std::static_pointer_cast<robot::Hand>(robots[0])->getFingerVel(i);
                Eigen::MatrixXd fing_jacob = std::static_pointer_cast<robot::Hand>(robots[0])->getFingerJacob(i);
                Eigen::VectorXd desDs = control::util::computeDsRBF(fing_pos, targetPositions[i],params.dsGain,params.dsRbfGain,params.dsMaxVel);
                dsController->update(fing_vel, desDs);
                Eigen::Vector4d taskTorque = fing_jacob.transpose() * dsController->get_output();
            
                if (i==0)
                    plotVariable = fing_pos -  targetPositions[i];
                // std::cout << "task command :" <<std::endl << taskTorque.transpose() << std::endl << std::endl; 
                //**null control
                Eigen::Vector4d nullTorque = control::util::getNullTorque(i,params.nullGain,nullPosition.segment(4*i,4),std::static_pointer_cast<robot::Hand>(robots[0])->finger[i].joint_states.pos,fing_jacob);               

                //** sum up
                cmdTorque.segment(4*i,4) = taskTorque + nullTorque;
            }
            //********compute the gravity compent
            Eigen::Matrix3d baseRotMat;
            baseRotMat = Eigen::AngleAxisd(-1*M_PI,Eigen::Vector3d::UnitX());
            std::static_pointer_cast<robot::Hand>(robots[0])->rotateGravity(baseRotMat);
            Eigen::VectorXd grvComp = std::static_pointer_cast<robot::Hand>(robots[0])->getCoriolisAndGravityForces(); 
            cmdTorque +=  grvComp;

            double EPSILON_FORCE = 1e-3;
            Eigen::VectorXd trqLim = std::static_pointer_cast<robot::Hand>(robots[0])->getTrqLimits();

            for (size_t i = 0; i < cmdTorque.size(); i++){
                if (fabs(cmdTorque[i]) < EPSILON_FORCE )
                    cmdTorque[i] = EPSILON_FORCE;
                else if ( cmdTorque[i] >  trqLim[i] )
                    cmdTorque[i] = trqLim[i];
                else if ( cmdTorque[i] <  -trqLim[i])
                    cmdTorque[i] = -trqLim[i];
            }

            std::vector<double> _data;
            
            for (size_t i = 0; i < 12; i++)
                _data.push_back(positions[i]);
            for (size_t i = 0; i < 16; i++)
                _data.push_back(impedances[i]);
            for (size_t i = 0; i < 12; i++)
                _data.push_back(cmdTorque[i]);
            
            data_log = _data;

            _step ++;
        }
    }
}

