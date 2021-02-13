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

        class PassiveDS
        {
        private:
            double eigVal0;
            double eigVal1;
            Eigen::Matrix3d damping_eigval = Eigen::Matrix3d::Identity();
            Eigen::Matrix3d baseMat = Eigen::Matrix3d::Identity();
        
            Eigen::Matrix3d Dmat = Eigen::Matrix3d::Identity();
            Eigen::Vector3d control_output = Eigen::Vector3d::Zero();
            void updateDampingMatrix(const Eigen::Vector3d& ref_vel);
        public:
            PassiveDS(const double& lam0, const double& lam1);
            ~PassiveDS();
            void set_damping_eigval(const double& lam0, const double& lam1);
            void update(const Eigen::Vector3d& vel, const Eigen::Vector3d& des_vel);
            Eigen::Vector3d get_output();
        };

        class Adaptive
        {
        private:
            Eigen::MatrixXd A_r;
            Eigen::MatrixXd B_r;
            Eigen::MatrixXd P_l;
            Eigen::MatrixXd Gamma;
            Eigen::VectorXd Imp;
            Eigen::MatrixXd Px;
            Eigen::MatrixXd Pr;
            bool is_impedance_active = false;

            double error_tol = 4e-1;
            double max_value = 2e2;


            Eigen::VectorXd output;
        public:
            Adaptive(const size_t& state_size);
            ~Adaptive();
            void activateImpedance(){is_impedance_active = true;};
            void deactivateImpedance(){is_impedance_active = false;};
            void setReferenceDyn(const Eigen::MatrixXd& Ar, const Eigen::MatrixXd& Br){A_r = Ar; B_r = Br;}
            void setReferenceA(const Eigen::MatrixXd& Ar){A_r = Ar;}
            void setReferenceB(const Eigen::MatrixXd& Br){B_r = Br;}

            void setAdaptiveGains(const Eigen::VectorXd& adGains){Gamma = adGains.asDiagonal();}
            void setImpedance(const Eigen::VectorXd& imp){ Imp = imp;}
            void resetAdaptation(){Px.setZero(); Pr.setZero();}

            void update(const Eigen::VectorXd& c_state, const Eigen::VectorXd& d_state);
            Eigen::VectorXd get_impedance();
            Eigen::VectorXd get_output(){return output;}

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