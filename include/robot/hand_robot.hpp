#ifndef __HANDTOOL_H__
#define __HANDTOOL_H__

// #include <iostream>
#include <fstream>
#include <memory>
#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>
#include "Utils.h"

#ifndef RBDL_BUILD_ADDON_URDFREADER
#error "Error: RBDL addon URDFReader not enabled."
#endif
#include <rbdl/addons/urdfreader/urdfreader.h>
#include "robot/abs_robot.hpp"

namespace robot{

    struct Finger
    {
        std::string name;                     // finger name
        std::string ee_name;
        unsigned int ee_id;
        //-Position Controller
        States task_states;
        States joint_states;
        Eigen::Matrix<double,3,4> jacobian;
        Eigen::Matrix<double,6,4> full_Jacobian;
        Eigen::Matrix3d rotMat;
    };
    class Hand : public AbsRobot{
    public:
        Hand();
        Hand(const std::string _urdf_filename);
        ~Hand(){
            delete rbdl_model;
        }
        // -------------- Update Hand Data
        void updateStates(const States& _states) override;
        void updateJntPos(const Eigen::VectorXd& pos) override;
        void updateJntVel(const Eigen::VectorXd& vel) override;
        void updateJntTrq(const Eigen::VectorXd& trq) override;

        // -------------- Set Gains and Control Params
        void rotateGravity(const Eigen::Matrix3d& baseRotation);

        // --------------- Perform forward dynamics
        Eigen::VectorXd perform_fdyn(Eigen::VectorXd Tau);      // getting the acceleration with respect to the torque

        // --------------- Get Hand Data
        Eigen::Vector3d getFingerPos(size_t i);
        Eigen::Vector4d getFingerOrientation(size_t i);
        Eigen::Vector3d getFingerAngVel(size_t i);
        Eigen::Matrix3d getFingerRotMat(size_t i);
        Eigen::MatrixXd getFingerFullJacobian(size_t i);
        Eigen::Vector3d getFingerVel(size_t i);
        Eigen::MatrixXd getFingerJacob(size_t i);
        Eigen::MatrixXd getMassMatrix();
        Eigen::VectorXd getCoriolisAndGravityForces();
        Eigen::VectorXd getGravityComp() override;
        Eigen::VectorXd getContactHessians(size_t i, const Eigen::Vector3d& relPoint = {0.,0.,0.});
        std::pair<Eigen::VectorXd, Eigen::VectorXd> getPosLimits();
        Eigen::VectorXd getVelLimits();
        Eigen::VectorXd getTrqLimits();
        size_t getNBfingers();

        Eigen::VectorXd getFingerInvKinematic(size_t i, const Eigen::VectorXd& pos);


        

        std::vector<Finger> finger;
      protected:
        // Fingers:
        size_t _fingersNum = 4;
        size_t _eachFingerDof = 4;
        std::string urdf_filename;
        RigidBodyDynamics::Model* rbdl_model = nullptr;

        // std::shared_ptr<RigidBodyDynamics::Model> rbdl_model;
        //=========================//
        void updateKinematicModel() override;
        void perform_fk(size_t f_indx, const Eigen::Vector3d& relPoint = {0.,0.,0.});
        Eigen::VectorXd perform_id();
        
    };
}


#endif

