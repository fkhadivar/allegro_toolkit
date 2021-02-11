//|
//|    Copyright (C) 2020 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
//|    Authors:  Farshad Khadivr (maintainer)
//|    email:   farshad.khadivar@epfl.ch
//|    website: lasa.epfl.ch
//|
//|    This file is part of alegro_toolkit.
//|
//|    alegro_toolkit is free software: you can redistribute it and/or modify
//|    it under the terms of the GNU General Public License as published by
//|    the Free Software Foundation, either version 3 of the License, or
//|    (at your option) any later version.
//|
//|    alegro_toolkit is distributed in the hope that it will be useful,
//|    but WITHOUT ANY WARRANTY; without even the implied warranty of
//|    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//|    GNU General Public License for more details.
//|

#include "motion_capture.h"


namespace environs{
        MotionCapture::MotionCapture(const size_t& num_objects,const double& _dt):dt(_dt){
            for(size_t i = 0; i < num_objects; i++) 
                objects.push_back(addObject());
        }
        object MotionCapture::addObject(){
            object _object;
            _object.pos = Eigen::Vector3d::Zero();
            _object.vel = Eigen::Vector3d::Zero();
            _object.acc = Eigen::Vector3d::Zero();
            _object.quat = Eigen::Vector4d::Zero();
            _object.quat[0] =1;
            _object.ang_pos =  Eigen::Vector3d::Zero();
            _object.ang_vel = Eigen::Vector3d::Zero();
            _object.ang_acc = Eigen::Vector3d::Zero();
            return _object;
        }

        void MotionCapture::setEntityStatic(const size_t& index){
            objects[index].type = "static";
        }
        size_t MotionCapture::defineEntity(const Eigen::Vector3d& pos, const Eigen::Vector4d& quat, const Eigen::Vector3d& vel,
        const Eigen::Vector3d& ang_vel){
            objects.push_back(addObject());
            size_t ind = objects.size() - 1;
            objects[ind].pos = pos;
            objects[ind].quat = quat;
            objects[ind].vel = vel;
            objects[ind].ang_vel = ang_vel;
            return ind;
        }

        void MotionCapture::updateEntity(const size_t& index, const Eigen::Vector3d& pos, const Eigen::Vector4d& orien){

            double fltr = filter_gain; 
            if(objects[index].type == "static")
                fltr = filter_gain_static;
            if(_isFirst){
                counter ++;
                objects[index].pos += pos;
                objects[index].quat += orien;
                if(counter > max_count){
                    _isFirst = false;
                    objects[index].pos = objects[index].pos *1/(max_count+1);
                    objects[index].quat = objects[index].quat *1/(max_count+1);
                }
            }else{
                _isOk = true; //todo improve the ok condition
                Eigen::Vector3d _p = 0.5*pos+ (1-0.5)*objects[index].pos;
                Eigen::Vector4d _q = fltr*orien+ (1-fltr)*objects[index].quat;
                _q.normalize();
                // if(index == 1){std::cout << "position: " << _p.transpose() << std::endl << std::endl;}

                //** this part to test raw data
                // if(index ==1){
                //     plotVar = (_p - objects[index].pos)*(1/dt);
                // }
                //**Position
                Eigen::Vector3d _v = fltr*(_p - objects[index].pos)*(1/dt)+ (1-fltr)*objects[index].vel;
                objects[index].acc = 0.5*fltr*(_v - objects[index].vel)*(1/dt)+ (1-0.5*fltr)*objects[index].acc;
                objects[index].vel = _v;
                objects[index].pos = _p;
                //** Orientation
                objects[index].quat= _q;
                objects[index].rotMat = Utils<double>::quaternionToRotationMatrix(objects[index].quat);
                Eigen::Vector3d axis = Eigen::Vector3d::Zero();
                double angle;
                Utils<double>::quaternionToAxisAngle(objects[index].quat, axis, angle);
                axis.normalize();
                Eigen::Vector3d axis_ang = angle * axis;
                Eigen::Vector3d _w = 0.5*fltr*(axis_ang - objects[index].ang_pos)*(1/dt) + (1-0.5*fltr)*objects[index].ang_vel;
                if (_w.norm()>50)
                    _w = objects[index].ang_vel;                
                objects[index].ang_acc = fltr*(_w - objects[index].ang_vel)*(1/dt)+ (1-fltr)*objects[index].ang_acc;
                objects[index].ang_vel = _w;
                objects[index].ang_pos = angle * axis;


                // if(index == 1){
                    // std::cout << "position: " << objects[index].pos.transpose() << std::endl << std::endl;
                    // std::cout << "velocity: " << objects[index].vel.transpose() << std::endl << std::endl;
                    // std::cout << "accelera: " << objects[index].acc.transpose() << std::endl << std::endl;
                // }

            }
         

        }
        environs::object MotionCapture::getEntity(const size_t& index){
            return objects[index];
        }
        environs::object MotionCapture::getRelativeEntity(const size_t& index_1, const size_t& index_0){
            object _obj;
            _obj.rotMat = objects[index_0].rotMat.transpose() * objects[index_1].rotMat; 
            _obj.quat   =  Utils<double>::rotationMatrixToQuaternion(_obj.rotMat);
            _obj.quat.normalize();
            Eigen::Vector3d axis = Eigen::Vector3d::Zero();
            double angle;
            Utils<double>::quaternionToAxisAngle(_obj.quat, axis, angle);
            axis.normalize();
            _obj.ang_pos= angle * axis;
            _obj.pos    =  objects[index_0].rotMat.transpose() * (objects[index_1].pos - objects[index_0].pos);
            _obj.vel    =  objects[index_0].rotMat.transpose() * (objects[index_1].vel - objects[index_0].vel);
            _obj.acc    =  objects[index_0].rotMat.transpose() * (objects[index_1].acc - objects[index_0].acc);
            _obj.ang_vel=  objects[index_0].rotMat.transpose() * (objects[index_1].ang_vel - objects[index_0].ang_vel);
            _obj.ang_acc=  objects[index_0].rotMat.transpose() * (objects[index_1].ang_acc - objects[index_0].ang_acc);

            return _obj;
        }


        bool MotionCapture::isOk(){
            return _isOk;
        }

        Eigen::Vector3d MotionCapture::getPlotVar(){
            return plotVar;
        }


}