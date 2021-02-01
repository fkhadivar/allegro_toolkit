#ifndef __ABSTRACT_Control__
#define __ABSTRACT_Control__
#include <fstream>
#include <Eigen/Dense>
#include "robot/abs_robot.hpp"
#include "control/control_util.h"

namespace control
{
    class AbsControl{
    public:
        AbsControl(/* args */){};
        virtual ~AbsControl(){};
        void addRobot(const std::shared_ptr<robot::AbsRobot>& _robot){
            robots.push_back(_robot);
        }
        //todo Other types of constructors if other entities exist
        //! or calling a function to set those entities
        virtual void setInput() = 0;
        virtual Eigen::VectorXd getOutput() =0;

    protected:
        virtual void algorithm() = 0;
        std::vector<std::shared_ptr<robot::AbsRobot>> robots;
    };
} // namespace control

#endif