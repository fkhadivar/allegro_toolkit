#include <mutex>
#include <fstream>
#include <pthread.h>
#include <memory>

#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"

#include "ros/ros.h"
#include <ros/package.h>
#include <Eigen/Dense>
#include "robot/hand_robot.hpp"
#include "control/ds_force_control.h"

// adding the config file
// and the dynamic reconfigurable things

#define NO_JOINTS 4
#define NO_FINGERS 4
#define FILTER_GAIN 0.2f

class AllegroRosMaster
{

public:
    AllegroRosMaster(ros::NodeHandle &n, double frequency,control::ds_force::Params controlParams):
    _n(n),_loopRate(frequency),_dt(1.0f/frequency),_controlParams(controlParams){
        _stop = false;
    }
    ~AllegroRosMaster(){};
    bool init(){    
        _allgeroHand =  std::make_shared<robot::Hand>(ros::package::getPath(std::string("allegro_toolkit")) + "/models/urdf/allegro_left.urdf");
        _controller = std::make_shared<control::ds_force::DsForceControl>(_allgeroHand,_controlParams);

        //** states
        _states.pos =Eigen::VectorXd::Zero(_allgeroHand->getDof());
        _states.vel =Eigen::VectorXd::Zero(_allgeroHand->getDof());
        _states.acc =Eigen::VectorXd::Zero(_allgeroHand->getDof());
        _states.trq =Eigen::VectorXd::Zero(_allgeroHand->getDof());
        
        _task_states.pos = Eigen::VectorXd::Zero(3*NO_FINGERS); 
        _task_states.vel = Eigen::VectorXd::Zero(3*NO_FINGERS);
        
        _cmd_states.pos.resize(3*NO_FINGERS); 
        _cmd_states.pos << .11,-.05,.1,
                           .11, .015,.1,
                           .11, .065,.1,
                           .12, -.06,.05;
        _cmd_states.vel =Eigen::VectorXd::Zero(_allgeroHand->getDof());
        _cmd_states.trq =Eigen::VectorXd::Zero(_allgeroHand->getDof());    
        
        //** communication:
        for (size_t i = 0; i < NO_FINGERS; i++){
            _subPosCmd[i] = _n.subscribe<std_msgs::Float64MultiArray>("/hand_0/finger_"+std::to_string(i)+"/pos_cmd",1,
                boost::bind(&AllegroRosMaster::updatePosCmd,this,_1,i),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
        }
             
        _subJointStates = _n.subscribe("/allegroHand_0/joint_states",1,
        &AllegroRosMaster::updateHandStates,this,ros::TransportHints().reliable().tcpNoDelay());

        for (size_t i = 0; i < NO_FINGERS; i++){
            _pubPosStates[i] =  _n.advertise<sensor_msgs::JointState>("/hand_0/finger_"+std::to_string(i)+"/pos_state",1);
            _pubJointStates[i] = _n.advertise<sensor_msgs::JointState>("/hand_0/finger_"+std::to_string(i)+"/joint_state",1);
        }
        
        _pubTorqueCmd = _n.advertise<sensor_msgs::JointState>("/allegroHand_0/torque_cmd",1);

        //
        return true;    
    }
    void run(){
        while(!_stop && ros::ok()){ 
            _mutex.lock();
            _allgeroHand->updateStates(_states);
            _controller->setInput(_cmd_states.pos.segment(0,3),_cmd_states.pos.segment(3,3),_cmd_states.pos.segment(6,3),_cmd_states.pos.segment(9,3));
            
            // _cmd_states.trq = _controller->getOutputTest();
            _cmd_states.trq = _controller->getOutput();

            for (size_t i = 0; i < NO_FINGERS; i++){
               _task_states.pos.segment(3*i,3) = _allgeroHand->getFingerPos(i);
               _task_states.vel.segment(3*i,3) = _allgeroHand->getFingerVel(i);
            }            
            publishHandStates(_cmd_states,_states,_task_states);
            
            _mutex.unlock();
            ros::spinOnce();
            _loopRate.sleep();
        }
    
        publishHandStates(_cmd_states,_states,_task_states);
        ros::spinOnce();
        _loopRate.sleep();
        ros::shutdown();
    }   
protected:
    ros::NodeHandle _n;
    ros::Rate _loopRate;
    double _dt;
    
    ros::Subscriber _subJointStates;
    ros::Subscriber _subPosCmd[NO_FINGERS];

    ros::Publisher _pubPosStates[NO_FINGERS];
    ros::Publisher _pubJointStates[NO_FINGERS];

    ros::Publisher _pubTorqueCmd;

    robot::States _states, _cmd_states, _task_states;

    bool _stop;
    std::mutex _mutex;

    std::shared_ptr<robot::Hand> _allgeroHand;
    std::shared_ptr<control::ds_force::DsForceControl> _controller;
    control::ds_force::Params _controlParams;
    Eigen::VectorXd _prevPos = Eigen::VectorXd::Zero(16);
    Eigen::VectorXd _prevVel = Eigen::VectorXd::Zero(16);

private:
    
    void updateHandStates(const sensor_msgs::JointState &msg){
        for (int i = 0; i < _allgeroHand->getDof(); i++){
            _states.pos[i] = (double)msg.position[i];
            _states.vel[i] = FILTER_GAIN * (_states.pos[i] - _prevPos[i])/_dt + (1 - FILTER_GAIN)*_prevVel[i];
            _states.trq[i] = (double)msg.effort[i];
        }
        _prevPos = _states.pos;
        _prevVel = _states.vel;
    }

    void updatePosCmd(const std_msgs::Float64MultiArray::ConstPtr &msg, int k){
       for (int i = 0; i < 3; i++){
            _cmd_states.pos[3*k+i] = (double)msg->data[i];
        }
        
    }

    void publishHandStates(const robot::States& cmd_states,const robot::States& joint_states,const robot::States& task_states){
        sensor_msgs::JointState _msgCmdStates;

        _msgCmdStates.position.resize(_allgeroHand->getDof());
        _msgCmdStates.velocity.resize(_allgeroHand->getDof());
        _msgCmdStates.effort.resize(_allgeroHand->getDof());
        for (int i = 0; i < _allgeroHand->getDof(); i++) {
            // _msgCmdStates.position[i] = (float)cmd_states.pos[i];
            _msgCmdStates.effort[i] = (float)cmd_states.trq[i];
        }        
        _pubTorqueCmd.publish(_msgCmdStates);
        
        std::vector<sensor_msgs::JointState> _msg_task_feedback(NO_FINGERS);
        std::vector<sensor_msgs::JointState> _msg_joint_feedback(NO_FINGERS);
        for (size_t i = 0; i < NO_FINGERS; i++){
            _msg_task_feedback[i].position.resize(3);
            _msg_task_feedback[i].velocity.resize(3);
            _msg_task_feedback[i].effort.resize(3);
            _msg_joint_feedback[i].position.resize(NO_JOINTS);
            _msg_joint_feedback[i].velocity.resize(NO_JOINTS);
            _msg_joint_feedback[i].effort.resize(NO_JOINTS);

            for (size_t j = 0; j < 3; j++){
                _msg_task_feedback[i].position[j] = (float)task_states.pos[3*i+j];
                _msg_task_feedback[i].velocity[j] = (float)task_states.vel[3*i+j];
                _msg_task_feedback[i].effort[j] = 0.f;             
            }
            
            for (size_t j = 0; j < NO_JOINTS; j++){
                _msg_joint_feedback[i].position[j] = (float)joint_states.pos[NO_JOINTS*i+j];
                _msg_joint_feedback[i].velocity[j] = (float)joint_states.vel[NO_JOINTS*i+j];
                _msg_joint_feedback[i].effort[j] = 0.f;             
            }

            _pubPosStates[i].publish(_msg_task_feedback[i]);
            _pubJointStates[i].publish(_msg_joint_feedback[i]);
        }

        
    }
};



int main(int argc, char **argv)
{
   float frequency = 200.0f;
   
    control::ds_force::Params controlParams;
    controlParams.dsGain = 5.;
    controlParams.dsMaxVel = .05;
    controlParams.dsRbfGain = 10.;
    controlParams.psvLambda = 5.;
    controlParams.psvDissip = .75;
    controlParams.nullGain = .1;

   ros::init(argc,argv,"finger_pos_controller");
   ros::NodeHandle n;  
   std::unique_ptr<AllegroRosMaster> AllegroMaster = std::make_unique<AllegroRosMaster>(n,frequency,controlParams);
   if (!AllegroMaster->init()){return -1;}else{AllegroMaster->run();}
   return 0;
}