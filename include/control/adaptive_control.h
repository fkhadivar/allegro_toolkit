#ifndef WHC_ADAPTIVE_CONTROL_H
#define WHC_ADAPTIVE_CONTROL_H


#include <memory>
#include <passive_ds_controller.h> //? do we need this
#include "Utils.h" //? do we need this
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
            std::vector<Eigen::MatrixXd> Px;
            std::vector<Eigen::MatrixXd> Pr;
            std::vector<Eigen::MatrixXd> Pf;
            // std::vector<Eigen::VectorXd> phi;
        };

        class ADControl : public AbsControl{
            /* data */
        public:
            ADControl(/* args */);
            ADControl(const std::shared_ptr<robot::AbsRobot>& _robot, Params& _params, double _dt);
            ~ADControl();
            /* inputs other than robot
                object
                optitrack
                biotac
            */
            void setInput() override;
            Eigen::VectorXd getOutput() override;
            Eigen::VectorXd getPlotVariable();

            void setParams(Params& _params);
        protected:
 
            //***********************
            void algorithm() override;
            void init_adaptive(const size_t& state_size);
            Eigen::VectorXd update_adaptive(const size_t& ind, const Eigen::VectorXd& c_state, const Eigen::VectorXd& d_state);
            Eigen::VectorXd joint_space_control(const size_t& ind, const Eigen::VectorXd& target_pos);
            Eigen::VectorXd task_space_control(const size_t& ind, const Eigen::VectorXd& target_pos);

            void setup_matrices_qp (const size_t& ind, const Eigen::VectorXd& _xdot_ref);
            double _step = 0 ;
            Params params;
            ad_Params ad_params;


            double dt;

            Eigen::VectorXd cmdTorque;
            Eigen::VectorXd plotVariable;

            std::shared_ptr<control::util::QP> _QP = nullptr;
        };

    }
}


#endif