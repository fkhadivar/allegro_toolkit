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

#ifndef WHC_QP_BASED_CONTROL_H
#define WHC_QP_BASED_CONTROL_H


#include <memory>
#include <passive_ds_controller.h> //? do we need this
#include "Utils.h" //? do we need this
#include "control/abs_control.hpp"
// #include "control/qp_solver.hpp"
#include "robot/hand_robot.hpp"

namespace control{
    namespace qp_control{

        struct Params
        {
            double dsGain, dsMaxVel;
            double psvLambda, psvDissip;
            double task_0, task_1;
        };

        class QPControl : public AbsControl{
            /* data */
        public:
            QPControl(/* args */);
            QPControl(const std::shared_ptr<robot::AbsRobot>& _robot, Params& _params, double _dt);
            ~QPControl();
            /* inputs other than robot
                object
                optitrack
                biotac
            */
            void setInput() override;
            Eigen::VectorXd getOutput() override;
            Eigen::VectorXd getPlotVariable();

            void setParams(Params& _params);
            void setNullPos(const Eigen::VectorXd& nullPose);
        protected:
            /* classical qp approach 
                input acceleration 
                output torque
            */
            void setup_matrices_invDyn(const Eigen::VectorXd& desired_acc, const Eigen::VectorXd& weights);
            void setup_matrices_ds(const Eigen::VectorXd& _fx, const Eigen::VectorXd& _tau_t, const Eigen::VectorXd& _tau_g, const Eigen::VectorXd& weight_0, const Eigen::VectorXd& weight_1);
            void setup_matrices_qp(const Eigen::VectorXd& _xddot_ref, const double& w_task, const double& w_reg, const double& w_out);
            
            //***********************
            void algorithm() override;
            double _step = 0 ;
            Params params;
            double dt;
            Eigen::VectorXd nullPosition;
            Eigen::VectorXd cmdTorque;
            Eigen::VectorXd plotVariable;

            std::vector<Eigen::MatrixXd> Kx;
            std::vector<Eigen::MatrixXd> Px;
            std::vector<Eigen::Matrix3d> Kr;
            std::vector<Eigen::MatrixXd> Pr;
            std::vector<Eigen::MatrixXd> Pf;
            std::vector<Eigen::Vector3d> phi;




            std::shared_ptr<control::util::QP> _QP = nullptr;
            // std::shared_ptr<DSController> dsController;
        };

    }
}


#endif