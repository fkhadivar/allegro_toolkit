#include <passive_ds_controller.h>
#include "Utils.h" //? do we need this
#include "control/abs_control.hpp"
#include "robot/hand_robot.hpp"

namespace control{
    namespace ds_force
    {
        struct Params
        {
            double dsGain, dsMaxVel, dsRbfGain;
            double psvLambda, psvDissip;
            double nullGain;
        };

        class DsForceControl : public AbsControl{
            /* data */
        public:
            DsForceControl(/* args */);
            DsForceControl(const std::shared_ptr<robot::AbsRobot>& _robot, Params& _params);
            // ~DsForceControl();
            /* inputs other than robot
                object
                optitrack
                biotac
            */
            void setInput() override;
            Eigen::VectorXd getOutput() override;
            Eigen::VectorXd getPlotVariable();

            void setParams(Params& _params);
            void setDsGain(const double dsGain);
            void setPsvGain(const double psvLambda, const double psvDissip);
            void setNullPos(const Eigen::VectorXd& nullPose);
        protected:
            void algorithm() override;

            void updateHandBasePose(const Eigen::Matrix3d& ref2Base); //? do we need this?
            double _step = 0 ;
            Params params;
            Eigen::VectorXd nullPosition;
            Eigen::VectorXd cmdTorque;
            Eigen::VectorXd plotVariable;
            std::shared_ptr<DSController> dsController;

        };

    } // namespace passive_ds
}


