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

#ifndef HAND_FT_CONTTOLLER_HPP
#define HAND_FT_CONTTOLLER_HPP


#include <memory>
#include "Utils.h" //? do we need this
#include "control/abs_control.hpp"
#include "control/qp_solver.hpp"
#include "robot/hand_robot.hpp"

namespace control{
    namespace qp_control{

        struct Params
        {
            double dsGain, dsMaxVel, dsMaxAngVel;
            double psvLambda, psvDissip;
            double task_0, task_1;


        };

        // struct EmgInput
        // {
        //     Eigen::Vector4i fingers_direction;
        //     Eigen::Vector3d rotAxis;
        //     double seq;
        // };

        class handFtController : public AbsControl{
            /* data */
            public:
                handFtController(/* args */);
                handFtController(const std::shared_ptr<robot::AbsRobot>& _robot, Params& _params, size_t _nbFingers, double _dt);
                
                /* inputs other than robot 
                    object
                    optitrack
                    biotac
                */
                void setInput() override;
                Eigen::VectorXd getOutput() override;
                Eigen::VectorXd getPlotVariable();
                
                void setParams(Params& _params);
                // void setEmgInput(EmgInput& _emgInput);
                void setNullPos(const Eigen::VectorXd& nullPose);
                void setTargetPose(std::vector<Eigen::Vector3d> targetPos, std::vector<Eigen::Vector4d> targetOrient);
                void setDesiredTorque(std::vector<Eigen::Vector3d> desTorque);
                void setGravityDirection(Eigen::Matrix3d GrotMat);
            protected:
                /* classical qp approach 
                    input acceleration 
                    output torque
                */
                Eigen::VectorXd computeDsManual(const Eigen::VectorXd& pos, const Eigen::VectorXd& desPos);
                void setup_matrices_invDyn(const Eigen::VectorXd& desired_acc, const Eigen::VectorXd& weights);
                void setup_matrices_ds(const Eigen::VectorXd& _fx, const Eigen::VectorXd& _tau_t, const Eigen::VectorXd& _tau_g, const Eigen::VectorXd& weight_0, const Eigen::VectorXd& weight_1);

                //***********************
                Eigen::VectorXd computeTorque(const std::vector<Eigen::VectorXd>& Poses);
                void algorithm() override;
                double _step = 0 ;
                Params params;
                size_t nbFingers;
                Eigen::Matrix3d baseRotMat;
                // EmgInput emgInput;
                double dt;
                Eigen::VectorXd nullPosition;
                Eigen::VectorXd cmdTorque;
                Eigen::VectorXd plotVariable;
                std::vector<Eigen::Vector3d> targetPositions;
                std::vector<Eigen::Vector4d> targetOrientations;
                std::vector<Eigen::Vector3d> desiredTorque;

                std::shared_ptr<control::util::QP> _QP = nullptr;

                std::shared_ptr<control::util::PassiveDS> dsController;
                std::shared_ptr<control::util::PassiveDS> dsController_orient;
        };

    }
}


#endif