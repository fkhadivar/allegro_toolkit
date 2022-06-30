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
#include <mutex>
#include <fstream>
#include <pthread.h>
#include "std_msgs/Bool.h"
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseStamped.h"
#include "ros/ros.h"
#include <ros/package.h>
#include <Eigen/Dense>
#include "robot/hand_robot.hpp"
#include "control/power_grasp_adaptive.h"


struct Options
{
    std::string control_mode;
    bool is_optitrack_on;
    double filter_gain = 0.2;
};
class HandRosMaster 
{
  public:
    HandRosMaster(ros::NodeHandle &n,double frequency, Options options, control::ad_control::Params controlParams):
    _n(n), _loopRate(frequency), _dt(1.0f/frequency),_options(options),_controlParams(controlParams){
        _stop =false;
        _graspCmd = false;
    }

    ~HandRosMaster(){}

    bool init(){
        
        _allgeroHand =  std::make_shared<robot::Hand>(ros::package::getPath(std::string("allegro_toolkit")) + "/models/urdf/allegro_left.urdf");

        _controller = std::make_shared<control::ad_control::ADPowerGrasp>(_allgeroHand,_controlParams,_dt);

        _states.pos =Eigen::VectorXd::Zero(_allgeroHand->getDof());
        _states.vel =Eigen::VectorXd::Zero(_allgeroHand->getDof());
        _states.acc =Eigen::VectorXd::Zero(_allgeroHand->getDof());
        _states.trq =Eigen::VectorXd::Zero(_allgeroHand->getDof());

        _cmd_states.pos =Eigen::VectorXd::Zero(_allgeroHand->getDof());
        _cmd_states.vel =Eigen::VectorXd::Zero(_allgeroHand->getDof());
        _cmd_states.trq =Eigen::VectorXd::Zero(_allgeroHand->getDof());


        _subJointSates = _n.subscribe("/allegroHand_0/joint_states",1,
        &HandRosMaster::updateHandStates,this,ros::TransportHints().reliable().tcpNoDelay());

        _subGraspCmd = _n.subscribe("/limbo/Grasp_command",1,
        &HandRosMaster::updateGraspCmd,this,ros::TransportHints().reliable().tcpNoDelay());

        _pubDesiredJointSates = _n.advertise<sensor_msgs::JointState>("desiredJointState",1);
        _pubJointCmd = _n.advertise<sensor_msgs::JointState>("/allegroHand_0/joint_cmd",  1);
        _pubTorqueCmd = _n.advertise<sensor_msgs::JointState>("/allegroHand_0/torque_cmd",1);



        return true;
    }
    // run node
    void run(){

        while(!_stop && ros::ok()){ 
            _mutex.lock();
        
            _allgeroHand->updateStates(_states);

            //todo send feedback to control class
            _controller->setCmd(_graspCmd);
            _cmd_states.trq = _controller->getOutput();
            publishHandStates(_cmd_states);

            _mutex.unlock();
            
            ros::spinOnce();
            _loopRate.sleep();
        }
    
        publishHandStates(_cmd_states);
        ros::spinOnce();
        _loopRate.sleep();
        ros::shutdown();
    }

  protected:
    robot::States _states, _cmd_states;
    double _dt;
    Options _options; 

    ros::NodeHandle _n;
    ros::Rate _loopRate;

    ros::Subscriber _subJointSates;                   // Joint States of Allegro Hand
    ros::Subscriber _subGraspCmd;                   // Joint States of Allegro Hand

    //ros::Subscriber _subJointCmd;                   // Joint Commands of Allegro Hand

    ros::Publisher _pubDesiredJointSates;         //  Joint States of Allegro Hand
    ros::Publisher _pubJointCmd;                  // Joint Commands of Allegro Hand for Position Mode
    ros::Publisher _pubTorqueCmd;                 // Torque Command for the Torque mode control
    
    bool _stop;                                         // Check for CTRL+C
    bool _graspCmd;
    std::mutex _mutex;


    std::shared_ptr<robot::Hand> _allgeroHand;
    std::shared_ptr<control::ad_control::ADPowerGrasp> _controller;
    control::ad_control::Params _controlParams;
    Eigen::VectorXd _prevPos = Eigen::VectorXd::Zero(16);
    Eigen::VectorXd _prevVel = Eigen::VectorXd::Zero(16);


  private:

    void updateHandStates(const sensor_msgs::JointState &msg){
        for (int i = 0; i < _allgeroHand->getDof(); i++){
            _states.pos[i] = (double)msg.position[i];
            _states.vel[i] = _options.filter_gain * (_states.pos[i] - _prevPos[i])/_dt + (1 - _options.filter_gain)*_prevVel[i];
            _states.trq[i] = (double)msg.effort[i];
        }
        _prevPos = _states.pos;
        _prevVel = _states.vel;
    }
    void updateGraspCmd(const std_msgs::Bool &msg){
        _graspCmd = msg.data;
    }
    // void updateBioTac(const biotac_sensors::BioTacHand &msg);

    void publishHandStates(const robot::States& cmd_states){
        sensor_msgs::JointState _msgCmdStates;
        _msgCmdStates.position.resize(_allgeroHand->getDof());
        _msgCmdStates.velocity.resize(_allgeroHand->getDof());
        _msgCmdStates.effort.resize(_allgeroHand->getDof());
        for (int i = 0; i < _allgeroHand->getDof(); i++) {
            _msgCmdStates.position[i] = (float)cmd_states.pos[i];
            _msgCmdStates.effort[i] = (float)cmd_states.trq[i];
        }
        if (_options.control_mode == "torque"){
            _pubTorqueCmd.publish(_msgCmdStates);
        }else{
            _pubJointCmd.publish(_msgCmdStates);
        }
    }


};


int main (int argc, char **argv)
{
    float frequency = 200.0f; //200.0f;
    Options options;
    options.control_mode = "torque";
    options.is_optitrack_on = false;
    options.filter_gain = .2;

    //todo the params for the qp to be added here
    control::ad_control::Params controlParams;

    //! change the parameters here:
    controlParams.dsGain = 5.;
    controlParams.dsMaxVel = .05;
    controlParams.psvLambda = 1.;
    controlParams.psvDissip = .5;
    controlParams.task_0 = 10.;
    controlParams.task_1 = 1.;

    ros::init(argc,argv, "hand_master");
    ros::NodeHandle n;
    std::shared_ptr<HandRosMaster> HandMaster = std::make_shared<HandRosMaster>(n,frequency,options,controlParams);

    if (!HandMaster->init()){
        return -1;
    }
    else{
        HandMaster->run();
    }
    return 0;
}