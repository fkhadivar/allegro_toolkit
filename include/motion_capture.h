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

#ifndef __MOTION_CAPTURE_H__
#define  __MOTION_CAPTURE_H__
#include <Eigen/Dense>
#include <memory>
#include <iostream>
#include "third_party/Utils.h"
namespace environs{

    struct object{
        std::string name;
        std::string type = "non_static";
        Eigen::Vector3d pos, vel, acc;
        Eigen::Vector4d quat;
        Eigen::Vector3d ang_pos, ang_vel, ang_acc;
        Eigen::Matrix3d rotMat = Eigen::Matrix3d::Identity();
    };
    class MotionCapture
    {
    private:
        bool _isOk = false;
        bool _isFirst = true;
        size_t counter = 0;
        const size_t max_count = 500;
        double filter_gain = 0.3;
        double filter_gain_static = 0.01;
        double dt = 0.005;
        Eigen::Vector3d plotVar = Eigen::Vector3d::Zero();
        environs::object addObject();
    public:
        MotionCapture(const size_t& num_objects, const double& _dt);
        ~MotionCapture(){};
        void setEntityStatic(const size_t& index);
        void updateEntity(const size_t& index, const Eigen::Vector3d& pos, const Eigen::Vector4d& orien);
        size_t defineEntity(const Eigen::Vector3d& pos, const Eigen::Vector4d& quat, 
            const Eigen::Vector3d& vel = Eigen::Vector3d::Zero(),const Eigen::Vector3d& ang_vel = Eigen::Vector3d::Zero());//todo
        environs::object getEntity(const size_t& index);
        environs::object getRelativeEntity(const size_t& index_1, const size_t& index_0);
        
        Eigen::Vector3d getPlotVar();

        bool isOk();
        std::vector<object> objects;
    };

}

#endif