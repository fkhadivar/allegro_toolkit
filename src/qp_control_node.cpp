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
#include "control/qp_based_control.h"


struct Options
{
    std::string control_mode;
    bool is_optitrack_on;
    double filter_gain = 0.;
};
class HandRosMaster 
{
  public:
    HandRosMaster(ros::NodeHandle &n,double frequency, Options options, control::qp_control::Params controlParams):
    _n(n), _loopRate(frequency), _dt(1.0f/frequency),_options(options),_controlParams(controlParams){
        _stop =false;

        //TODO clean the optitrack code
        _optitrack_initiated = true;
        _optitrack_ready = true;
        if(_options.is_optitrack_on){
            _optitrack_initiated = false;
            _optitrack_ready = false;
        }

    }

    ~HandRosMaster(){}

    bool init(){
        
        _allgeroHand =  std::make_shared<robot::Hand>("/home/farshad/catkin_allegro/src/Hand_Master/Hand_Master/models/urdf/allegro_left.urdf");
        //todo reading params from yaml

        // _controller = std::make_unique<control::qp_control::QPControl>(_allgeroHand,_controlParams,_dt);
        _controller = std::make_shared<control::qp_control::QPControl>(_allgeroHand,_controlParams,_dt);

        _states.pos =Eigen::VectorXd::Zero(_allgeroHand->getDof());
        _states.vel =Eigen::VectorXd::Zero(_allgeroHand->getDof());
        _states.acc =Eigen::VectorXd::Zero(_allgeroHand->getDof());
        _states.trq =Eigen::VectorXd::Zero(_allgeroHand->getDof());

        _cmd_states.pos =Eigen::VectorXd::Zero(_allgeroHand->getDof());
        _cmd_states.vel =Eigen::VectorXd::Zero(_allgeroHand->getDof());
        _cmd_states.trq =Eigen::VectorXd::Zero(_allgeroHand->getDof());


        _subJointSates = _n.subscribe("/allegroHand_0/joint_states",1,
        &HandRosMaster::updateHandStates,this,ros::TransportHints().reliable().tcpNoDelay());
        _subOptitrack[0] = _n.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/baseHand/pose", 1,
            boost::bind(&HandRosMaster::updateOptitrack,this,_1,0),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
        // _subOptitrack[1] = _n.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/baseHand2/pose", 1,
        //    boost::bind(&HandRosMaster::updateOptitrack,this,_1,1),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
        _subOptitrack[1] = _n.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/objectHand2/pose", 1,
            boost::bind(&HandRosMaster::updateOptitrack,this,_1,1),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());

        _pubDesiredJointSates = _n.advertise<sensor_msgs::JointState>("desiredJointState",1);
        _pubJointCmd = _n.advertise<sensor_msgs::JointState>("/allegroHand_0/joint_cmd",  1);
        _pubTorqueCmd = _n.advertise<sensor_msgs::JointState>("/allegroHand_0/torque_cmd",1);

        // plotting
        _plotVar.data.resize(3);
        //TODO make plotting more clean and like a function that receives input from contrl unit
        _plotter = _n.advertise<std_msgs::Float64MultiArray>("/hand/plotvar",1);



        
        //todo condition here
        return true;
    }
    // run node
    void run(){

        while(!_stop && ros::ok()){ 
            if (_optitrack_initiated){
                _mutex.lock();
                if(_optitrack_ready){
                    _allgeroHand->updateStates(_states);

                    //todo send feedback to control class
                    // _controller->setInput();
                    _cmd_states.trq = _controller->getOutput();
                    publishHandStates(_cmd_states);

                    Eigen::VectorXd plotVariable = _controller->getPlotVariable();
                    for (size_t i = 0; i < 3; i++)
                        _plotVar.data[i] = plotVariable[i];
                    _plotter.publish(_plotVar);
                    //logData
                
                }else{ optitrackInitialization(); }
                _mutex.unlock();
            }
        ros::spinOnce();
        _loopRate.sleep();
        }
    
        publishHandStates(_cmd_states);
        ros::spinOnce();
        _loopRate.sleep();
        // _outputFile.close();
        ros::shutdown();
    }

  protected:
    robot::States _states, _cmd_states;
    double _dt;
    Options _options; 

    ros::NodeHandle _n;
    ros::Rate _loopRate;

    ros::Subscriber _subJointSates;                   // Joint States of Allegro Hand
    //ros::Subscriber _subJointCmd;                   // Joint Commands of Allegro Hand
    ros::Subscriber _subOptitrack[2];  // optitrack markers pose //todo clean here
    // ros::Subscriber _subBioTac;

    ros::Publisher _pubDesiredJointSates;         //  Joint States of Allegro Hand
    ros::Publisher _pubJointCmd;                  // Joint Commands of Allegro Hand for Position Mode
    ros::Publisher _pubTorqueCmd;                 // Torque Command for the Torque mode control
    
    std_msgs::Float64MultiArray _plotVar;
    ros::Publisher _plotter;

    bool _optitrack_initiated;         // Monitor first optitrack markers update
    bool _optitrack_ready;                                  // Check if all markers position is received
    bool _stop;                                         // Check for CTRL+C
    std::mutex _mutex;

    std::shared_ptr<robot::Hand> _allgeroHand;
    // std::unique_ptr<control::qp_control::QPControl> _controller;
    std::shared_ptr<control::qp_control::QPControl> _controller;
    control::qp_control::Params _controlParams;
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

    //TODO clean the optitrack
    void optitrackInitialization(){

    }
    void updateOptitrack(const geometry_msgs::PoseStamped::ConstPtr& msg, int k){

    }
    uint16_t checkTrackedMarker(float a, float b){

    }

};


int main (int argc, char **argv)
{
    float frequency = 100.0f; //200.0f;
    Options options;
    options.control_mode = "torque";
    options.is_optitrack_on = false;
    options.filter_gain = .2;

    //todo the params for the qp to be added here
    control::qp_control::Params controlParams;

    controlParams.dsGain = 5.;
    controlParams.dsMaxVel = .05;
    controlParams.psvLambda = 1.;
    controlParams.psvDissip = .5;
    controlParams.task_0 = 10.;
    controlParams.task_1 = 1.;

    ros::init(argc,argv, "hand_master");
    ros::NodeHandle n;
    // std::string filename;
    // ROS_INFO("[debug]: Here 1 ...");
    // you can use some if statements for having diffrent modes
    // std::unique_ptr<HandRosMaster> HandMaster = std::make_unique<HandRosMaster>(n,frequency,options,controlParams);
    std::shared_ptr<HandRosMaster> HandMaster = std::make_shared<HandRosMaster>(n,frequency,options,controlParams);

    if (!HandMaster->init()){
        return -1;
    }
    else{
        HandMaster->run();
    }
    return 0;
}