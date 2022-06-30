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

#ifndef WHC_ADAPTIVE_CONTROL_H
#define WHC_ADAPTIVE_CONTROL_H


#include <memory>
// #include "Utils.h" //? do we need this
#include "control/abs_control.hpp"
// #include "control/qp_solver.hpp"
#include "robot/hand_robot.hpp"

namespace control{
    namespace ad_control{

        struct Params
        {
            double dsGain, dsMaxVel;
            double psvLambda, psvDissip;
            double task_0, task_1;
        };
        struct ad_Params
        {
            Eigen::MatrixXd A_r;
            Eigen::MatrixXd B_r;
            Eigen::MatrixXd P_l;
            Eigen::MatrixXd Gamma;
            Eigen::VectorXd Imp;
            std::vector<Eigen::MatrixXd> Px;
            std::vector<Eigen::MatrixXd> Pr;
            std::vector<Eigen::MatrixXd> Pf;
            // std::vector<Eigen::VectorXd> phi;
        };

        class ADPowerGrasp : public AbsControl{
            /* data */
        public:
            ADPowerGrasp(/* args */);
            ADPowerGrasp(const std::shared_ptr<robot::AbsRobot>& _robot, Params& _params, double _dt);
            ~ADPowerGrasp();
            /* inputs other than robot
                object
                optitrack
                biotac
            */
            void setCmd(bool cmd);
            
            void setInput() override;
            Eigen::VectorXd getOutput() override;

            void setParams(Params& _params);
        protected:
            std::vector<double> data_log;
            //***********************
            void algorithm() override;
            Eigen::VectorXd joint_space_control(const size_t& ind, const Eigen::VectorXd& target_pos);

            double _step = 0 ;
            Params params;

            double dt;
            bool do_grasp;
            Eigen::VectorXd cmdTorque;
            std::vector<std::shared_ptr<control::util::Adaptive>> _AD;

        };

    }
}


#endif