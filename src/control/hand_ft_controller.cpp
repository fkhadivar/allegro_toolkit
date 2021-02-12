
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

#include "control/hand_ft_controller.hpp"


namespace control{
    namespace qp_control{

        handFtController::handFtController(const std::shared_ptr<robot::AbsRobot>& _robot, Params& _params, size_t _nbFingers, double _dt):params(_params), dt(_dt), nbFingers(_nbFingers){
            addRobot(_robot);
            nullPosition =  Eigen::VectorXd::Zero(robots[0]->getDof());
            _QP = std::make_shared<control::util::QP>();
            dsController = std::make_shared<control::util::PassiveDS>(params.psvLambda, params.psvDissip* params.psvLambda);
            dsController_orient = std::make_shared<control::util::PassiveDS>(params.psvLambda, params.psvDissip* params.psvLambda);
            cmdTorque = Eigen::VectorXd::Zero(16);

            targetPositions = std::vector<Eigen::Vector3d>(nbFingers);
            targetOrientations = std::vector<Eigen::Vector4d>(nbFingers);
            desiredTorque = std::vector<Eigen::Vector3d>(nbFingers);
            for(size_t i=0; i<nbFingers; i++){
                desiredTorque[i] = Eigen::Vector3d::Zero(3);
            }
            // baseRotMat = Eigen::AngleAxisd(0,Eigen::Vector3d::UnitY());
            baseRotMat = Eigen::Matrix3d::Identity(3, 3);
        }
        void handFtController::setInput(){}
        void handFtController::setParams(Params& _params){params = _params;}
        // void handFtController::setEmgInput(EmgInput& _emgInput){emgInput =_emgInput;}
        void handFtController::setNullPos(const Eigen::VectorXd& nullPose){nullPosition = nullPose;}

        Eigen::VectorXd handFtController::getOutput() {
            algorithm();
            //todo std:pair 
            return cmdTorque;
        }

        Eigen::VectorXd handFtController::getPlotVariable(){
            return plotVariable;
        }

        Eigen::VectorXd handFtController::computeDsManual(const Eigen::VectorXd& pos, const Eigen::VectorXd& desPos){
                // Eigen::VectorXd deltaX = desPos - pos;
                Eigen::VectorXd deltaX = desPos.head(3) - pos.head(3);
                if (deltaX.norm() > params.dsMaxVel){
                    deltaX = params.dsMaxVel * deltaX.normalized();
                }
                // std::cout << "pos: " << pos.transpose() << std::endl;
                // std::cout << "desPos: " << desPos.transpose() << std::endl;
                Eigen::Vector4d dqd = Utils<double>::slerpQuaternion(pos.tail(4), desPos.tail(4), 0.2);

                Eigen::Vector4d deltaQ = Eigen::Vector4d::Zero(4);

                deltaQ = dqd - pos.tail(4);

                Eigen::Vector4d qconj = pos.tail(4);
                qconj.segment(1,3) = -1 * qconj.segment(1,3);
                Eigen::Vector4d temp_angVel = Utils<double>::quaternionProduct(deltaQ, qconj);

                Eigen::Vector3d tmp_angular_vel = temp_angVel.segment(1,3);
                if (tmp_angular_vel.norm() > params.dsMaxAngVel){
                    tmp_angular_vel = params.dsMaxAngVel * tmp_angular_vel.normalized();
                }


                // compute desired velocity from a non-linear dynamical system (using a rbf kernel)
                // linear velocity
                Eigen::VectorXd rbf_vector =  Eigen::VectorXd::Zero(3);
                rbf_vector.head(3) =  deltaX;
                Eigen::MatrixXd Sigma_position = 1/(params.dsMaxVel*params.dsMaxVel) * Eigen::MatrixXd::Identity(3,3);
                double thetagain = -0.5 * rbf_vector.transpose() * Sigma_position * rbf_vector;
                Eigen::VectorXd desLinVel = params.dsGain*(1. + std::exp(thetagain)) * deltaX;


                // angualar velocity
                rbf_vector = tmp_angular_vel;
                Eigen::MatrixXd Sigma_orientation = 1/(params.dsMaxAngVel*params.dsMaxAngVel) * Eigen::MatrixXd::Identity(3,3);
                thetagain = -0.5 * rbf_vector.transpose() * Sigma_orientation * rbf_vector;
                Eigen::VectorXd desAngVel = params.dsGain*(1. + std::exp(thetagain)) * tmp_angular_vel;

                Eigen::VectorXd desVel(desLinVel.size() + desAngVel.size());
                desVel << desAngVel, desLinVel;
                // std::cout << "desVel: " << desVel.transpose() << std::endl;
                return desVel;
        }

        //***********************************************

        void handFtController::setup_matrices_ds(const Eigen::VectorXd& _fx, const Eigen::VectorXd& _tau_t, const Eigen::VectorXd& _tau_g,
         const Eigen::VectorXd& weight_0, const Eigen::VectorXd& weight_1){
            //**[dot{q}(16)  tau(16)]
            size_t dofs = robots[0]->getDof();
            size_t _dim =  2*dofs;
            // size_t ntaskspace =  12;
            size_t ntaskspace =  24;

            //** Setup Task
            std::vector<Eigen::MatrixXd> A_t;
            std::vector<Eigen::VectorXd> b_t;
            std::vector<Eigen::VectorXd> w_t;
            std::vector<Eigen::MatrixXd> A_c;
            std::vector<Eigen::MatrixXd> b_c;

            w_t.push_back(weight_0);
            w_t.push_back(weight_1);

            A_t.push_back(Eigen::MatrixXd::Zero(ntaskspace, _dim));
            // std::cout << "A_t: " << A_t[0].rows() << ", " << A_t[0].cols() << std::endl;
            b_t.push_back(Eigen::VectorXd::Zero(ntaskspace));
            // std::cout << "b_t: " << b_t[0].rows() << ", " << b_t[0].cols() << std::endl;

            Eigen::MatrixXd Jacobs = Eigen::MatrixXd::Zero(ntaskspace, dofs);
            
            for (size_t i = 0; i < 4; i++){
                // Jacobs.block(3*i,4*i,3,4) = std::static_pointer_cast<robot::Hand>(robots[0])->finger[i].jacobian;
                Jacobs.block(6*i,4*i,6,4) = std::static_pointer_cast<robot::Hand>(robots[0])->finger[i].full_Jacobian;
            }
            
            A_t[0].block(0, 0 ,Jacobs.rows(), Jacobs.cols()) = Jacobs;
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


            // Variable bounds
            Eigen::VectorXd lb = Eigen::VectorXd::Zero(_dim);
            Eigen::VectorXd ub = Eigen::VectorXd::Zero(_dim);
            lb.head(dofs) = -std::static_pointer_cast<robot::Hand>(robots[0])->getVelLimits();
            ub.head(dofs) =  std::static_pointer_cast<robot::Hand>(robots[0])->getVelLimits();
            lb.segment(dofs, dofs) =-_tau_t -_tau_g - std::static_pointer_cast<robot::Hand>(robots[0])->getTrqLimits();
            ub.segment(dofs, dofs) =-_tau_t -_tau_g + std::static_pointer_cast<robot::Hand>(robots[0])->getTrqLimits();

            _QP->setup_qp_tasks(A_t,b_t,w_t);
            _QP->setup_qp_constraints(A_c,b_c);
            _QP->setup_qp_bounds(lb,ub);

        }
        //**=================================================
        //**=================================================
        
        
        Eigen::VectorXd handFtController::computeTorque(const std::vector<Eigen::VectorXd>& Poses){
            size_t dofs = robots[0]->getDof();
            Eigen::VectorXd _trq = Eigen::VectorXd::Zero(dofs);
            //*******get force from passive Ds
            size_t numfingers = std::static_pointer_cast<robot::Hand>(robots[0])->finger.size();
            std::vector<Eigen::Vector4d> tasktorques(4);
            Eigen::VectorXd nullPosition =  Eigen::VectorXd::Zero(robots[0]->getDof()); // todo move this to the hand
            nullPosition[12] = 1.3;
            nullPosition[13] = -.1;
            nullPosition[14] = .1;
            nullPosition[15] = .1;

            // Eigen::VectorXd fx = Eigen::VectorXd::Zero(12);
            Eigen::VectorXd fx = Eigen::VectorXd::Zero(numfingers*6);
            for (size_t i = 0; i < numfingers ; i++){
                
                Eigen::Vector3d fing_pos = std::static_pointer_cast<robot::Hand>(robots[0])->finger[i].task_states.pos;
                Eigen::Vector4d fing_orient = std::static_pointer_cast<robot::Hand>(robots[0])->getFingerQuat(i);
                Eigen::MatrixXd fJacob = std::static_pointer_cast<robot::Hand>(robots[0])->getFingerFullJacobian(i);
                Eigen::MatrixXd fing_jacob = std::static_pointer_cast<robot::Hand>(robots[0])->finger[i].jacobian;
                //** get the Ds
                Eigen::Vector3d fing_vel = std::static_pointer_cast<robot::Hand>(robots[0])->finger[i].task_states.vel;
                Eigen::Vector3d fing_AngVel = std::static_pointer_cast<robot::Hand>(robots[0])->getFingerAngVel(i);
                
                Eigen::Matrix3d fing_RotMat = std::static_pointer_cast<robot::Hand>(robots[0])->getFingerRotMat(i);
                Eigen::VectorXd finger_pose(fing_pos.size() + fing_orient.size());
                finger_pose << fing_pos, fing_orient;
                Eigen::VectorXd desiredVel = computeDsManual(finger_pose, Poses[i]);

                // compute the desired linear force coming out of passive ds
                dsController->update(fing_vel, desiredVel.tail(3));
                
                Eigen::VectorXd linear_force = dsController->get_output();

                // compute the desired coriolis velocity coming out of passive ds
                dsController_orient->update(fing_AngVel, desiredVel.head(3));
                Eigen::Vector3d cor_force = dsController_orient->get_output();

                Eigen::VectorXd des_force = Eigen::VectorXd::Zero(desiredVel.size());
                des_force << 0.1*cor_force, linear_force + fing_RotMat.transpose()*desiredTorque[i];
                // std::cout << "des_force: " << des_force.transpose() << std::endl;
                // tasktorques[i] = fing_jacob.transpose() * des_force.tail(3);
                tasktorques[i] = fJacob.transpose() * des_force;
                
                // tasktorques[i] = fing_jacob.transpose() * (dsController->get_output() + fing_RotMat.transpose()*desiredTorque[i]);
                // fx.segment(3*i,3) = desiredVel.tail(3);
                fx.segment(6*i, 6) = desiredVel;
                //**null control
                Eigen::Vector4d temNullTorque = Eigen::Vector4d::Zero();
                for (size_t j = 0; j < _trq.size()/numfingers; j++){
                    temNullTorque[j] = -.1 * (std::static_pointer_cast<robot::Hand>(robots[0])->finger[i].joint_states.pos[j]  - nullPosition[j+4*i]);
                }
                Eigen::MatrixXd tempMat = fing_jacob * fing_jacob.transpose();
                Eigen::Matrix4d nullMat =  Eigen::MatrixXd::Identity(4,4) - fing_jacob.transpose() * tempMat.inverse() * fing_jacob;
                Eigen::Vector4d nullTorque = nullMat * temNullTorque;
                if (nullTorque[0] != nullTorque[0]){
                    nullTorque = Eigen::Vector4d::Zero();
                }
                //** sum up
                _trq.segment(4*i,4) = tasktorques[i];// + nullTorque;
                if (i==0)
                    plotVariable = fing_pos -  Poses[i].head(3);
            }

            //********compute the gravity compent
            Eigen::VectorXd grvComp = std::static_pointer_cast<robot::Hand>(robots[0])->getCoriolisAndGravityForces(); 

            Eigen::VectorXd weight_0 = Eigen::VectorXd::Constant(24,params.task_0);
            Eigen::VectorXd weight_1 = Eigen::VectorXd::Constant(16,params.task_1);
            setup_matrices_ds(fx, _trq, grvComp, weight_0, weight_1);
            Eigen::VectorXd sol;
            // sol.resize(2*dofs+12);
            sol.resize(2*dofs);
            if (_QP->qp_solve())
                sol = _QP->qp_solution();

            _trq +=  grvComp + sol.segment(dofs,dofs);
            if (_step < 100)
                _trq = Eigen::VectorXd::Zero(dofs);
            return _trq;
        }


        void handFtController::setTargetPose(std::vector<Eigen::Vector3d> targetPos, std::vector<Eigen::Vector4d> targetOrient){
            targetPositions = targetPos;
            targetOrientations = targetOrient;
        }

        void handFtController::setDesiredTorque(std::vector<Eigen::Vector3d> desTorque){
            desiredTorque = desTorque;
        }

        void handFtController::setGravityDirection(Eigen::Matrix3d GrotMat){
            baseRotMat = GrotMat;
        }


        void handFtController::algorithm(){
            

            // double regionMargin = 0.06;
            // Eigen::Vector3d X_Object_inRef, objectGraspPosition;
            // objectGraspPosition =  {0.14,-0.04, 0.08};
            // double graspZ = 0.005;
            // double graspY = 0.005;
            // // grasp pose in hand frame
            // std::vector<Eigen::Vector3d> graspPositions(4);
            // graspPositions[0] =  Eigen::Vector3d(.0,    -graspY, graspZ);
            // graspPositions[1] =  Eigen::Vector3d(.0,     graspY, graspZ);
            // graspPositions[2] =  Eigen::Vector3d(.0,    .05,    .0);
            // graspPositions[3] =  Eigen::Vector3d(-0.02, .0,     -graspZ);
            // // open pose in hand frame
            // std::vector<Eigen::Vector3d> openPositions(4);
            // openPositions[0] = Eigen::Vector3d(0.075 , -0.05 , 0.2);
            // openPositions[1] = Eigen::Vector3d(0.075 ,  0.00 , 0.2);
            // openPositions[2] = Eigen::Vector3d(0.075 ,  0.05 , 0.2);
            // openPositions[3] = Eigen::Vector3d(0.075 ,  -0.02 , 0.0);


           
            //** Define Hand Orientation
             // the direction of gravity (gravity vector) 
            // baseRotMat = Eigen::AngleAxisd(-1*M_PI,Eigen::Vector3d::UnitY());
            
            std::static_pointer_cast<robot::Hand>(robots[0])->rotateGravity(baseRotMat);

            std::vector<Eigen::VectorXd> targetPoses(nbFingers);

            for (size_t i=0; i<nbFingers; i++){
                targetPoses[i] = Eigen::VectorXd(targetPositions[i].size() + targetOrientations[i].size());
                targetPoses[i] << targetPositions[i], targetOrientations[i];
            }

            // Eigen::VectorXd targetPose(targetPositions.size() + targetOrientations.)

            cmdTorque = computeTorque(targetPoses);

            _step ++;
        }

    }
}

