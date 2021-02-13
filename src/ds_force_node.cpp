//** FARSHAD

#include <mutex>
#include <fstream>
#include <pthread.h>
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseStamped.h"
#include "ros/ros.h"
#include <ros/package.h>
#include <Eigen/Dense>
#include "robot/hand_robot.hpp"
#include "control/ds_force_control.h"

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

struct Options
{
    std::string control_mode;
    bool is_optitrack_on;
    double filter_gain = 0.2;
};
class HandRosMaster 
{
  public:
    HandRosMaster(ros::NodeHandle &n,double frequency, Options options, control::ds_force::Params controlParams):
    _n(n), _loopRate(frequency), _dt(1.0f/frequency),_options(options),_controlParams(controlParams){
        _stop =false;

    }

    ~HandRosMaster(){}

    bool init(){
        _allgeroHand =  std::make_shared<robot::Hand>(ros::package::getPath(std::string("allegro_toolkit")) + "/models/urdf/allegro_left.urdf");
        // ! initialize control Params
        //todo reading params from yaml

        _controller = std::make_shared<control::ds_force::DsForceControl>(_allgeroHand,_controlParams);

        _states.pos =Eigen::VectorXd::Zero(_allgeroHand->getDof());
        _states.vel =Eigen::VectorXd::Zero(_allgeroHand->getDof());
        _states.acc =Eigen::VectorXd::Zero(_allgeroHand->getDof());
        _states.trq =Eigen::VectorXd::Zero(_allgeroHand->getDof());

        _cmd_states.pos =Eigen::VectorXd::Zero(_allgeroHand->getDof());
        _cmd_states.vel =Eigen::VectorXd::Zero(_allgeroHand->getDof());
        _cmd_states.trq =Eigen::VectorXd::Zero(_allgeroHand->getDof());


        _subJointSates = _n.subscribe("/allegroHand_0/joint_states",1,
        &HandRosMaster::updateHandStates,this,ros::TransportHints().reliable().tcpNoDelay());

        _pubDesiredJointSates = _n.advertise<sensor_msgs::JointState>("desiredJointState",1);
        _pubJointCmd = _n.advertise<sensor_msgs::JointState>("/allegroHand_0/joint_cmd",  1);
        _pubTorqueCmd = _n.advertise<sensor_msgs::JointState>("/allegroHand_0/torque_cmd",1);

        // plotting
        _plotVar.data.resize(3);
        //TODO make plotting more clean and like a function that receives input from contrl unit
        _plotter = _n.advertise<std_msgs::Float64MultiArray>("/hand/plotvar",1);

        std::string _fileName = "passiveds_stability_rot_2";
        _outputFile.open(ros::package::getPath(std::string("allegro_toolkit"))+"/data_recording/"+_fileName+".csv");
        if(!_outputFile.is_open()){ ROS_ERROR("[Master]: Cannot open output file, the data directory might be missing");
            return false;
        }
        std::vector<std::string> header = _controller->get_data_header();
        for (size_t i = 0; i < header.size(); i++)
            _outputFile << header[i] << ", ";
        _outputFile <<"\n";

        
        //todo condition here
        return true;
    }
    // run node
    void run(){

        while(!_stop && ros::ok()){ 
            
            _mutex.lock();
            
            _allgeroHand->updateStates(_states);

            //todo send feedback to control class
            // _controller->setInput();
            _cmd_states.trq = _controller->getOutput();
            publishHandStates(_cmd_states);

            Eigen::VectorXd plotVariable = _controller->getPlotVariable();
            for (size_t i = 0; i < 3; i++)
                _plotVar.data[i] = plotVariable[i];
            _plotter.publish(_plotVar);
            logData(_controller->get_data_log());
            
            _mutex.unlock();
            ros::spinOnce();
            _loopRate.sleep();
        }
    
        publishHandStates(_cmd_states);
        ros::spinOnce();
        _loopRate.sleep();
        _outputFile.close();
        ros::shutdown();
    }

  protected:
    robot::States _states, _cmd_states;
    double _dt;
    Options _options; 
    std::ofstream _outputFile;

    ros::NodeHandle _n;
    ros::Rate _loopRate;

    ros::Subscriber _subJointSates;                   // Joint States of Allegro Hand
    //ros::Subscriber _subJointCmd;                   // Joint Commands of Allegro Hand

    ros::Publisher _pubDesiredJointSates;         //  Joint States of Allegro Hand
    ros::Publisher _pubJointCmd;                  // Joint Commands of Allegro Hand for Position Mode
    ros::Publisher _pubTorqueCmd;                 // Torque Command for the Torque mode control
    
    std_msgs::Float64MultiArray _plotVar;
    ros::Publisher _plotter;

    bool _stop;                                         // Check for CTRL+C
    std::mutex _mutex;

    std::shared_ptr<robot::Hand> _allgeroHand;
    // std::unique_ptr<control::ds_force::DsForceControl> _controller;
    std::shared_ptr<control::ds_force::DsForceControl> _controller;
    control::ds_force::Params _controlParams;
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

    void logData(const std::vector<double>& _data){
        for (size_t i = 0; i < _data.size(); i++)
            _outputFile << _data[i] << ", ";
        _outputFile <<"\n";
    }

};


int main (int argc, char **argv)
{
    float frequency = 200.0f; //200.0f;
    Options options;
    options.control_mode = "torque";
    options.is_optitrack_on = false;
    options.filter_gain = .2;

    control::ds_force::Params controlParams;
    controlParams.dsGain = 5.;
    controlParams.dsMaxVel = .05;
    controlParams.dsRbfGain = 10.;
    controlParams.psvLambda = 5.;
    controlParams.psvDissip = .75;
    controlParams.nullGain = .1;
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