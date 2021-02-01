#ifndef _CONTROL_UTIL_
#define _CONTROL_UTIL_
#include "Utils.h" //? do we need this
#include <fstream>
#include <Eigen/Dense>
#include "control/qp_solver.hpp"



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