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

#ifndef __ABSTRACT_Control__
#define __ABSTRACT_Control__
#include <fstream>
#include <Eigen/Dense>
#include <vector>
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