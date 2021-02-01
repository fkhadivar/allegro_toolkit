

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


struct Options
{
    std::string control_mode;
    double filter_gain = 0.;

};


class HandRosMaster 
{
  public:
    HandRosMaster(ros::NodeHandle &n,double frequency, Options options, control::qp_control::Params controlParams, std::string _hand_urdf):
    _n(n), _loopRate(frequency), _dt(1.0f/frequency),_options(options),_controlParams(controlParams), hand_urdf(_hand_urdf){
        _stop =false;
        
    }

    ~HandRosMaster(){}

    bool init(){
        
        _allgeroHand =  std::make_shared<robot::Hand>(hand_urdf);
        //todo reading params from yaml

        // _controller = std::make_unique<control::qp_control::EmgControl>(_allgeroHand,_controlParams,_dt);
        _controller = std::make_shared<control::qp_control::handFtController>(_allgeroHand, _controlParams, _allgeroHand->getNBfingers(), _dt);

        _states.pos =Eigen::VectorXd::Zero(_allgeroHand->getDof());
        _states.vel =Eigen::VectorXd::Zero(_allgeroHand->getDof());
        _states.acc =Eigen::VectorXd::Zero(_allgeroHand->getDof());
        _states.trq =Eigen::VectorXd::Zero(_allgeroHand->getDof());

        _cmd_states.pos = Eigen::VectorXd::Zero(_allgeroHand->getDof());
        _cmd_states.vel = Eigen::VectorXd::Zero(_allgeroHand->getDof());
        _cmd_states.trq = Eigen::VectorXd::Zero(_allgeroHand->getDof());

        _targetPosition = std::vector<Eigen::Vector3d>(_allgeroHand->getNBfingers());
        _targetOrientation = std::vector<Eigen::Vector4d>(_allgeroHand->getNBfingers());

        // initialize gravity vector to point downwards
        _gravityVector << 0.0f, 0.0f, -1.0f;

        // initialize bendingness and speed gains to be 1
        _bendingnessVector = Eigen::Vector4f::Ones();
        _ftSpeedGrains = Eigen::Vector4f::Ones();


        _subJointSates = _n.subscribe("/allegroHand_0/joint_states", 1, &HandRosMaster::updateHandStates, this,ros::TransportHints().reliable().tcpNoDelay());
        _subTarget = _n.subscribe("/allegroHand_0/ft_pose_cmd", 1, &HandRosMaster::updateTargetPose, this,ros::TransportHints().reliable().tcpNoDelay());
        _subTorque = _n.subscribe("/allegroHand_0/ft_torque_cmd", 1, &HandRosMaster::updateDesTorque, this,ros::TransportHints().reliable().tcpNoDelay());
        _subGravVector = _n.subscribe("/allegroHand_0/Gravity_direction", 1, &HandRosMaster::updateGravityDirection, this,ros::TransportHints().reliable().tcpNoDelay());
        _subBendingness = _n.subscribe("/allegroHand_0/Bendingness", 1, &HandRosMaster::updateBendingness, this,ros::TransportHints().reliable().tcpNoDelay());
        _subFTspeed = _n.subscribe("/allegroHand_0/SpeedGains", 1, &HandRosMaster::updateSpeedGains, this,ros::TransportHints().reliable().tcpNoDelay());

        _pubDesiredJointSates = _n.advertise<sensor_msgs::JointState>("/allegroHand_0/desiredJointState",1);
        _pubJointCmd = _n.advertise<sensor_msgs::JointState>("/allegroHand_0/joint_cmd",  1);
        _pubTorqueCmd = _n.advertise<sensor_msgs::JointState>("/allegroHand_0/torque_cmd", 1);
        _pubftPose = _n.advertise<geometry_msgs::PoseArray>("/allegroHand_0/ft_pos", 1);

        _ftPose_msg.poses.resize(_allgeroHand->getNBfingers());
        _ftPose_msg.header.frame_id = "[INDEX, MIDDLE, RING, THUMB]";

        // plotting
        _plotVar.data.resize(3);
        //TODO make plotting more clean and like a function that receives input from contrl unit
        _plotter = _n.advertise<std_msgs::Float64MultiArray>("/hand/plotvar", 1);

        _firstRobotStatesReceived = false;

        ROS_INFO("[Hand_FT_Controller] Waiting for the first hand states ...");
        while (!_stop && !_firstRobotStatesReceived){
            ros::spinOnce();
            _loopRate.sleep();
        }
        
        if (_firstRobotStatesReceived){
            ROS_INFO("[Hand_FT_Controller] Hand states received!");
        
            // update the robot states
            _allgeroHand->updateStates(_states);

            for (size_t fngr=0; fngr<_allgeroHand->getNBfingers(); fngr++){
                _targetPosition[fngr] = _allgeroHand->getFingerPos(fngr);
                _targetOrientation[fngr] = _allgeroHand->getFingerOrientation(fngr);
            }

            ROS_INFO("[Hand_FT_Controller] FT position controller successfully initialized!");
            ROS_INFO("[Hand_FT_Controller] Ready to receive position commands.");

            return true;
        }else{
            return false;
        }
        
        
    }

    // run node
    void run(){

        while(!_stop && ros::ok()){ 

            _mutex.lock();

            // update the robot states
            _allgeroHand->updateStates(_states);

            // set the desired target pose
            _controller->setTargetPose(_targetPosition, _targetOrientation);

            // compute desired torque
            _cmd_states.trq = _controller->getOutput();
            
            // publish command
            publishHandStates(_cmd_states);

            publishFTpose();

            _mutex.unlock();


            
            // if (_optitrack_initiated){
            //     _mutex.lock();
            //     if(_optitrack_ready){

            //         _allgeroHand->updateStates(_states);

            //         //todo send feedback to control class
            //         _controller->setEmgInput(_emgInput);
            //         // _controller->setInput();
            //         _cmd_states.trq = _controller->getOutput();
            //         publishHandStates(_cmd_states);

            //         Eigen::VectorXd plotVariable = _controller->getPlotVariable();
            //         for (size_t i = 0; i < 3; i++)
            //             _plotVar.data[i] = plotVariable[i];
            //         _plotter.publish(_plotVar);
            //         //logData
                
            //     }else{ optitrackInitialization(); }
            //     _mutex.unlock();
            // }
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
    // control::qp_control::EmgInput _emgInput;
    std::string hand_urdf;

    ros::NodeHandle _n;
    ros::Rate _loopRate;

    ros::Subscriber _subJointSates;                     // Joint States of Allegro Hand
    ros::Subscriber _subTarget;                         // Target pose of the fingertips
    ros::Subscriber _subTorque;                         // Target torque of the fingertips
    ros::Subscriber _subGravVector;                     // Gravity vector
    ros::Subscriber _subFTspeed;                        // the speed of the fingertips
    ros::Subscriber _subBendingness;                    // the "bendingness" of the fingers

    ros::Publisher _pubDesiredJointSates;               // Joint States of Allegro Hand
    ros::Publisher _pubJointCmd;                        // Joint Commands of Allegro Hand for Position Mode
    ros::Publisher _pubTorqueCmd;                       // Torque Command for the Torque mode control
    ros::Publisher _pubftPose;                          // publisher of the fingertips position

    geometry_msgs::PoseArray _ftPose_msg;

    std::vector<Eigen::Vector3d> _targetPosition;       // desired target position
    std::vector<Eigen::Vector4d> _targetOrientation;    // desired target orientation
    Eigen::Vector3f _gravityVector;                     // gravity direction
    Eigen::Vector4f _bendingnessVector;                 // how much each finger could be bend
    Eigen::Vector4f _ftSpeedGrains;                     // speed gains for each finger

    
    std_msgs::Float64MultiArray _plotVar;
    ros::Publisher _plotter;

    bool _stop;                                         // Check for CTRL+C
    bool _firstRobotStatesReceived;
    std::mutex _mutex;

    std::shared_ptr<robot::Hand> _allgeroHand;
    // std::unique_ptr<control::qp_control::EmgControl> _controller;
    std::shared_ptr<control::qp_control::handFtController> _controller;
    control::qp_control::Params _controlParams;
    Eigen::VectorXd _prevPos = Eigen::VectorXd::Zero(16);
    Eigen::VectorXd _prevVel = Eigen::VectorXd::Zero(16);
    


  private:

    void updateHandStates(const sensor_msgs::JointState &msg){

        if (!_firstRobotStatesReceived){
            _firstRobotStatesReceived = true;
        }
        for (int i = 0; i < _allgeroHand->getDof(); i++){
            _states.pos[i] = (double)msg.position[i];
            _states.vel[i] = _options.filter_gain * (_states.pos[i] - _prevPos[i])/_dt + (1 - _options.filter_gain)*_prevVel[i];
            _states.trq[i] = (double)msg.effort[i];
        }
        _prevPos = _states.pos;
        _prevVel = _states.vel;
    }
    

    void updateTargetPose(const geometry_msgs::PoseArray &msg){
        /*
         *  callback function for listening to the desired pose of the fingertips
         *  
         */
        for (size_t fngr=0; fngr<_targetPosition.size(); fngr++){
            _targetPosition[fngr] << msg.poses[fngr].position.x, msg.poses[fngr].position.y, msg.poses[fngr].position.z;
            _targetOrientation[fngr] << msg.poses[fngr].orientation.w, msg.poses[fngr].orientation.x, msg.poses[fngr].orientation.y, msg.poses[fngr].orientation.z;
        }
    }

    void updateDesTorque(const geometry_msgs::PoseArray &msg){
        /*
         *  callback function for listening to the desired torque for the fingertips
         *  
         */
        std::vector<Eigen::Vector3d> desTorque(_allgeroHand->getNBfingers());

        std::cout << "Test\n";

        for (size_t fngr=0; fngr<desTorque.size(); fngr++){
            desTorque[fngr] << msg.poses[fngr].position.x, msg.poses[fngr].position.y, msg.poses[fngr].position.z;
        }
        std::cout << desTorque[0].transpose() << std::endl;
        _controller->setDesiredTorque(desTorque);
    }


    void updateGravityDirection(const std_msgs::Float32MultiArray& msg){
        /*
         *  callback function for listening to the gravity direction
         *  
         */
        for (size_t i=0; i<_gravityVector.size(); i++){
            _gravityVector(i) = msg.data[i];
        }
    }


    void updateSpeedGains(const std_msgs::Float32MultiArray& msg){
        /*
         *  callback function for listening to the speed gains
         *  
         */
        for (size_t i=0; i<_ftSpeedGrains.size(); i++){
            _ftSpeedGrains(i) = msg.data[i];
        }
    }


    void updateBendingness(const std_msgs::Float32MultiArray& msg){
        /*
         *  callback function for listening to the bending parameters
         *  
         */
        for (size_t i=0; i<_bendingnessVector.size(); i++){
            _bendingnessVector(i) = msg.data[i];
        }
    }


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
            std::cout << "I am publishing sth" << std::endl;
        }else{
            _pubJointCmd.publish(_msgCmdStates);
        }
    }

    void publishFTpose(){
        /*
         *  function for publishing the current pose of the fingertips
         * 
         */
        for (size_t fngr=0; fngr<_allgeroHand->getNBfingers(); fngr++){

            // get position orientation for each of the finger from fk
            Eigen::Vector3d ft_postion = _allgeroHand->getFingerPos(fngr);
            Eigen::Vector4d ft_orientation = _allgeroHand->getFingerOrientation(fngr);

            // assign the position and orientation values to the message
            _ftPose_msg.poses[fngr].position.x = ft_postion(0);
            _ftPose_msg.poses[fngr].position.y = ft_postion(1);
            _ftPose_msg.poses[fngr].position.z = ft_postion(2);

            _ftPose_msg.poses[fngr].orientation.w = ft_orientation(0);
            _ftPose_msg.poses[fngr].orientation.x = ft_orientation(1);
            _ftPose_msg.poses[fngr].orientation.y = ft_orientation(2);
            _ftPose_msg.poses[fngr].orientation.z = ft_orientation(3);
        }

        _pubftPose.publish(_ftPose_msg);
    }

    //TODO clean the optitrack
    // void optitrackInitialization(){

    // }
    // void updateOptitrack(const geometry_msgs::PoseStamped::ConstPtr& msg, int k){

    // }
    // uint16_t checkTrackedMarker(float a, float b){

    // }

//     void updateTargetPose(const emg_interface::emg_msg &msg){
//         /
//         ROS_INFO("[handmaster] I received new fingertips pose..");
        
//         for (int i = 0; i < _options.nb_fingers; i++){
//             _emgInput.fingers_direction[i] = msg.Fingers_Direction[i];
//         }

//         for (int i = 0; i < _options.nb_rot_axis; i++){
//             _emgInput.rotAxis[i] = msg.Rotation_Axis[i];
//         }

//         _emgInput.seq = msg.Seq;
//     //     desired_gain = msg.Gain;

//     //     for(int i =0; i< NB_Fingers; i++){
//     //         if (emg_fing_dir[i] == _desired_fingers_direction[i]){
//     //             stepCounter[i] +=1;
//     //         }else{
//     //             stepCounter[i] = 0;
//     //         }

//     //         emg_fing_dir[i] = _desired_fingers_direction[i];

//     //     }
//     }

};


int main (int argc, char **argv)
{
    float frequency = 100.0f; //200.0f;
    Options options;
    options.control_mode = "torque";
    // options.is_optitrack_on = false;
    options.filter_gain = .2;
    // options.is_emg_on = true;
    // options.nb_fingers = 4;
    // options.nb_rot_axis = 3;

    //todo the params for the qp to be added here
    control::qp_control::Params controlParams;

    controlParams.dsGain = 6.; // max 5
    controlParams.dsMaxVel = .05;
    controlParams.dsMaxAngVel = .02; // 0.2
    controlParams.psvLambda = 7.; //max 2
    controlParams.psvDissip = .5;
    controlParams.task_0 = 10.;
    controlParams.task_1 = 1.;

    std::string hand_description;
    

    ros::init(argc,argv, "hand_ft_controller");
    ros::NodeHandle n;
    if (argc > 1){
        hand_description = std::string(argv[1]);
        std::cout << "[Hand_FT_Controller] " << hand_description<< std::endl;
    }else{
        ROS_INFO("[Hand_FT_Controller] No URDF file with hand description provided");
        ROS_INFO("[Hand_FT_Controller] Shutting down the node");
        return -1;
    }
    // std::string filename;
    // ROS_INFO("[debug]: Here 1 ...");
    // you can use some if statements for having diffrent modes
    // std::unique_ptr<HandRosMaster> HandMaster = std::make_unique<HandRosMaster>(n,frequency,options,controlParams);
    std::shared_ptr<HandRosMaster> HandMaster = std::make_shared<HandRosMaster>(n,frequency, options, controlParams, hand_description);

    if (!HandMaster->init()){
        ROS_INFO("[Hand_FT_Controller] Shutting down the node");
        return -1;
    }else{
        HandMaster->run();
    }
    return 0;
}