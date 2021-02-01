

#include <mutex>
#include <fstream>
#include <pthread.h>
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "ros/ros.h"
#include <ros/package.h>
#include <Eigen/Dense>
#include "robot/hand_robot.hpp"
#include "control/hand_ft_controller.hpp"

#include "third_party/sg_filter.h"


struct Options
{
    std::string control_mode;
    double filter_gain = 0.;

};


class HandRosMaster 
{
  public:
    HandRosMaster(ros::NodeHandle &n,double frequency, Options options, std::string _hand_urdf):
    _n(n), _loopRate(frequency), _dt(1.0f/frequency), _options(options), hand_urdf(_hand_urdf){
        _stop =false;
        
    }

    ~HandRosMaster(){}

    bool init(){
        
        _allegroHand = std::make_shared<robot::Hand>(hand_urdf);
        

        _states.pos =Eigen::VectorXd::Zero(_allegroHand->getDof());
        _states.vel =Eigen::VectorXd::Zero(_allegroHand->getDof());
        _states.acc =Eigen::VectorXd::Zero(_allegroHand->getDof());
        _states.trq =Eigen::VectorXd::Zero(_allegroHand->getDof());

        _cmd_states.pos = Eigen::VectorXd::Zero(_allegroHand->getDof());
        _cmd_states.vel = Eigen::VectorXd::Zero(_allegroHand->getDof());
        _cmd_states.trq = Eigen::VectorXd::Zero(_allegroHand->getDof());

        desiredTorques = Eigen::VectorXd::Zero(_allegroHand->getDof());


        _subJointSates = _n.subscribe("/allegroHand_0/joint_states", 1, &HandRosMaster::updateHandStates, this,ros::TransportHints().reliable().tcpNoDelay());
        _subTorqueCmd = _n.subscribe("/allegroHand_0/torque_cmd", 1, &HandRosMaster::updateTorqueCmd, this,ros::TransportHints().reliable().tcpNoDelay());
        
        _pubJointCmd = _n.advertise<sensor_msgs::JointState>("/allegroHand_0/joint_cmd",  1);
        

        filt_order = 2;
        winlength = 5;//(int)_allegroHand->getDof() + 1;
        filt_dim = _allegroHand->getDof();

        sg_filter = SGF::SavitzkyGolayFilter(filt_dim, filt_order, winlength, (float)_dt);

        _firstRobotStatesReceived = false;

        ROS_INFO("[Hand_Torque_Sim] Waiting for the first hand states ...");
        while (!_stop && !_firstRobotStatesReceived){
            ros::spinOnce();
            _loopRate.sleep();
        }
        
        if (_firstRobotStatesReceived){
            ROS_INFO("[Hand_Torque_Sim] Hand states received!");
        
            // update the robot states
            _allegroHand->updateStates(_states);

            ROS_INFO("[Hand_Torque_Sim] Ready to receive torque commands.");

            return true;
        }else{
            return false;
        }
        
        
    }

    // run node
    void run(){

        int counter = 0;

        while(!_stop && ros::ok()){ 

            _mutex.lock();

            // std::cout << counter << std::endl;

            // update the robot states
            _allegroHand->updateStates(_states);


            // compute next joint states based on the received torque command
            computeNextState();

            // publish the desired joint states
            publishHandStates(_cmd_states);

            _mutex.unlock();

            ros::spinOnce();
            _loopRate.sleep();

            counter ++;
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
    std::string hand_urdf;

    ros::NodeHandle _n;
    ros::Rate _loopRate;

    ros::Subscriber _subJointSates;                     // Joint States of Allegro Hand
    ros::Subscriber _subTorqueCmd;                      // listening to torque commands
    
    ros::Publisher _pubJointCmd;                        // Joint Commands of Allegro Hand for Position Mode

    Eigen::VectorXd desiredTorques;                     // desired torques as received from the topic

    bool _stop;                                         // Check for CTRL+C
    bool _firstRobotStatesReceived;
    std::mutex _mutex;

    std::shared_ptr<robot::Hand> _allegroHand;
    Eigen::VectorXd _prevPos = Eigen::VectorXd::Zero(16);
    Eigen::VectorXd _prevVel = Eigen::VectorXd::Zero(16);

    SGF::SavitzkyGolayFilter sg_filter;
    int filt_order, winlength, filt_dim;
    


  private:

    void updateHandStates(const sensor_msgs::JointState &msg){

        if (!_firstRobotStatesReceived){
            _firstRobotStatesReceived = true;
        }
        for (int i = 0; i < _allegroHand->getDof(); i++){
            _states.pos[i] = (double)msg.position[i];
            _states.vel[i] = _options.filter_gain * (_states.pos[i] - _prevPos[i])/_dt + (1 - _options.filter_gain)*_prevVel[i];
            _states.trq[i] = (double)msg.effort[i];
        }
        _prevPos = _states.pos;
        _prevVel = _states.vel;
    }

    void updateTorqueCmd(const sensor_msgs::JointState &msg){
        for (int i = 0; i < _allegroHand->getDof(); i++){
            desiredTorques(i) = (double)msg.effort[i];
        }
    }
    


    void computeNextState(){

        // desiredTorques = Eigen::VectorXd::Zero(16);

        Eigen::VectorXd qdd = _allegroHand->perform_fdyn(desiredTorques);

        // std::cout << "des torques: " << desiredTorques.transpose() << std::endl;

        Eigen::VectorXf toBeFiltered(qdd.size());

        for (size_t i=0; i<qdd.size(); i++){
            _cmd_states.vel[i] = _dt*qdd[i]; //+ _prevVel[i];
            _cmd_states.pos[i] = _dt*_cmd_states.vel[i] + _prevPos[i];
            toBeFiltered(i) = (float)_cmd_states.pos[i];
            _cmd_states.trq[i] = desiredTorques[i];
        }

        // Eigen::VectorXd toBeFiltered (_cmd_states.pos.size());

        sg_filter.AddData(toBeFiltered);
        

        Eigen::VectorXf filteredSignal;

        if (sg_filter.GetOutput(0, filteredSignal) >= 0){
            for (size_t i=0; i<qdd.size(); i++){
                _cmd_states.pos[i] = (double)filteredSignal(i);
            }
        }

        // std::cout << "qdd: " << qdd.transpose() << std::endl;
    }


    void publishHandStates(const robot::States& cmd_states){
        sensor_msgs::JointState _msgCmdStates;
        _msgCmdStates.position.resize(_allegroHand->getDof());
        _msgCmdStates.velocity.resize(_allegroHand->getDof());
        _msgCmdStates.effort.resize(_allegroHand->getDof());
        for (int i = 0; i < _allegroHand->getDof(); i++) {
            _msgCmdStates.position[i] = (float)cmd_states.pos[i];
            _msgCmdStates.effort[i] = (float)cmd_states.trq[i];
        }
        _pubJointCmd.publish(_msgCmdStates);
    }

};


int main (int argc, char **argv)
{
    float frequency = 100.0f; //200.0f;
    Options options;
    options.control_mode = "torque";
    options.filter_gain = .2;

    //todo the params for the qp to be added here
    control::qp_control::Params controlParams;

    controlParams.dsGain = 4.; // max 5
    controlParams.dsMaxVel = .05;
    controlParams.psvLambda = 1.; //max 2
    controlParams.psvDissip = .5;
    controlParams.task_0 = 10.;
    controlParams.task_1 = 1.;

    std::string hand_description;
    

    ros::init(argc,argv, "hand_ft_controller");
    ros::NodeHandle n;
    if (argc > 1){
        hand_description = std::string(argv[1]);
        std::cout << "[Hand_Torque_Sim] " << hand_description<< std::endl;
    }else{
        ROS_INFO("[Hand_Torque_Sim] No URDF file with hand description provided");
        ROS_INFO("[Hand_Torque_Sim] Shutting down the node");
        return -1;
    }
    
    std::shared_ptr<HandRosMaster> HandMaster = std::make_shared<HandRosMaster>(n,frequency, options, hand_description);

    if (!HandMaster->init()){
        ROS_INFO("[Hand_Torque_Sim] Shutting down the node");
        return -1;
    }else{
        HandMaster->run();
    }
    return 0;
}