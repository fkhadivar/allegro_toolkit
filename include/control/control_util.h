#ifndef _CONTROL_UTIL_
#define _CONTROL_UTIL_
#include "Utils.h" //? do we need this
#include <fstream>
#include <Eigen/Dense>
#include "control/qp_solver.hpp"


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

namespace control{
    namespace util{
        
        class QP
        {
        protected:
            /* data */
            std::shared_ptr<control::qp_solver::QPOases> _solver = nullptr;
            Eigen::MatrixXd _H, _A;
            Eigen::VectorXd _g, _ub, _lb, _ubA, _lbA;
        public:
            QP(/* args */);
            ~QP();
            void setup_qp_tasks(std::vector<Eigen::MatrixXd>& A_t, std::vector<Eigen::VectorXd>& b_t,std::vector<Eigen::VectorXd>& w_t);
            void setup_qp_constraints(std::vector<Eigen::MatrixXd>& A_c, std::vector<Eigen::MatrixXd>& b_c);
            void setup_qp_bounds(Eigen::VectorXd& lb, Eigen::VectorXd& ub);
            bool qp_solve();
            Eigen::VectorXd qp_solution() const;
        }; 

        Eigen::VectorXd computeDs(const Eigen::VectorXd& pos, const Eigen::VectorXd& desPos,
                                        const double& dsGain = 1., const double& maxDx = .05);
        Eigen::VectorXd computeDsRBF(const Eigen::VectorXd& pos, const Eigen::VectorXd& desPos,
                                        const double& dsGain = 1., const double& dsRbfGain = 1., const double& maxDx = .05,  const double& lambda = 1.);
        double computeRbfGain(const Eigen::VectorXd& x, const Eigen::VectorXd& xd,
                                        const double& maxGain, const double& minGain, const double& maxDx);

        Eigen::VectorXd getNullTorque(const size_t& ind, const double& nullGain, const Eigen::Vector4d& nullpose,
                                    const Eigen::Vector4d& robpose, const Eigen::MatrixXd& fing_jacob);
        double smoothVel(const Eigen::Vector3d& vel, const double& minGain,const double& sigma);
    } // namespace util
} // namespace control

#endif