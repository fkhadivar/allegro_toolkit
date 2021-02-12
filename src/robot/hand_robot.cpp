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

#include "robot/hand_robot.hpp"


namespace robot{
    Hand::Hand(const std::string _urdf_filename):urdf_filename(_urdf_filename){

        for (size_t i = 0; i< _fingersNum; i++){
            finger.push_back(Finger());
            finger[i].name = "N";
            finger[i].task_states.pos = Eigen::Vector3d::Zero();
            finger[i].task_states.vel = Eigen::Vector3d::Zero();
            finger[i].task_states.acc = Eigen::Vector3d::Zero();
            finger[i].task_states.trq = Eigen::Vector3d::Zero();
            finger[i].joint_states.pos = Eigen::Vector4d::Zero();
            finger[i].joint_states.vel = Eigen::Vector4d::Zero();
            finger[i].joint_states.acc = Eigen::Vector4d::Zero();
            finger[i].joint_states.trq = Eigen::Vector4d::Zero();
            finger[i].jacobian = Eigen::MatrixXd::Identity(3,4);
            finger[i].full_Jacobian = Eigen::MatrixXd::Zero(6,4);
        }
        // for the left hand
        finger[0].name = "index";
        finger[1].name = "midle";
        finger[2].name = "pinky";
        finger[3].name = "thumb";

        finger[0].ee_name = "link_0_3_tip";
        finger[1].ee_name = "link_1_3_tip";
        finger[2].ee_name = "link_2_3_tip";
        finger[3].ee_name = "link_3_3_tip";

        //todo initiate the rbdl solver
        rbdl_check_api_version (RBDL_API_VERSION);
        // rbdl_model = std::make_shared<RigidBodyDynamics::Model>();
        rbdl_model = new RigidBodyDynamics::Model();
        // Load the urdf model
        if (!RigidBodyDynamics::Addons::URDFReadFromFile (urdf_filename.c_str(),rbdl_model, false)) {
            std::cerr << "Error loading model " << std::endl;
            abort();
        }
        std::cout << "Degree of freedom overview:" << std::endl;
        std::cout << RigidBodyDynamics::Utils::GetModelDOFOverview(*rbdl_model);
        std::cout << "Model Hierarchy:" << std::endl;
        std::cout << RigidBodyDynamics::Utils::GetModelHierarchy(*rbdl_model);

        setDof(rbdl_model->q_size);
        for (size_t i = 0; i < _fingersNum; i++)
            finger[i].ee_id = rbdl_model->GetBodyId(finger[i].ee_name.c_str());

        
    }

    //====================== Input =======================//
    void Hand::updateStates(const States& _states){
            checkDim(_states.pos);
            robot_states = _states;
            for (size_t i = 0; i < _fingersNum; i++)
                for (size_t j = 0; j < _eachFingerDof; j++){
                    finger[i].joint_states.pos(j) = robot_states.pos(_eachFingerDof*i+j);
                    finger[i].joint_states.vel(j) = robot_states.vel(_eachFingerDof*i+j);
                    finger[i].joint_states.acc(j) = robot_states.acc(_eachFingerDof*i+j);
                    finger[i].joint_states.trq(j) = robot_states.trq(_eachFingerDof*i+j);
                }
            updateKinematicModel();
        }
    void Hand::updateJntPos(const Eigen::VectorXd& pos){
        checkDim(pos);
        robot_states.pos = pos;
        for (size_t i = 0; i < _fingersNum; i++)
            for (size_t j = 0; j < _eachFingerDof; j++)
                finger[i].joint_states.pos(j) = pos(_eachFingerDof*i+j);
        updateKinematicModel();
    }

    void Hand::updateJntVel(const Eigen::VectorXd& vel){
        checkDim(vel);
        robot_states.vel = vel;
        for (size_t i = 0; i < _fingersNum; i++)
            for (size_t j = 0; j < _eachFingerDof; j++)
                finger[i].joint_states.vel(j) = vel(_eachFingerDof*i+j);
        updateKinematicModel();
    }

    void Hand::updateJntTrq(const Eigen::VectorXd& trq){
        checkDim(trq);
        robot_states.trq = trq;
        for (size_t i = 0; i < _fingersNum; i++)
            for (size_t j = 0; j < _eachFingerDof; j++)
                finger[i].joint_states.trq(j) = trq(_eachFingerDof*i+j);
        //todo putting the id updating here
        //! Updare ID
    }
    
    std::pair<Eigen::VectorXd, Eigen::VectorXd>Hand::getPosLimits(){
        //! this has to be cleaned and be received from the libraray
        Eigen::VectorXd pos_limits_u; pos_limits_u.resize(rbdl_model->q_size);
        pos_limits_u << 0.54, 1.75, 1.709, 1.75, 
                        0.54, 1.75, 1.709, 1.75, 
                        0.54, 1.75, 1.709, 1.75, 1.58, 1.26, 1.66,1.719; 
        Eigen::VectorXd pos_limits_l; pos_limits_l.resize(rbdl_model->q_size);
        pos_limits_l << -0.62, -0.26, -0.28, -0.3, 
                        -0.62, -0.26, -0.28, -0.3, 
                        -0.62, -0.26, -0.28, -0.3, 0.263, -0.30, -0.26, -0.2; 
        return std::make_pair(pos_limits_l,pos_limits_u);
    }
    Eigen::VectorXd Hand::getVelLimits(){
        //! this has to be cleaned and be received from the libraray
        return Eigen::VectorXd::Constant(16,7); 
    }
    Eigen::VectorXd Hand::getTrqLimits(){
        //! this has to be cleaned and be received from the libraray
        return Eigen::VectorXd::Constant(16,10.); 
    }

    Eigen::MatrixXd Hand::getMassMatrix() {
            Eigen::MatrixXd H = Eigen::MatrixXd::Zero(rbdl_model->q_size,rbdl_model->q_size);
            RigidBodyDynamics::CompositeRigidBodyAlgorithm(*rbdl_model, robot_states.pos,H, false);
            return H;
    }
    Eigen::VectorXd Hand::getCoriolisAndGravityForces(){
        rbdl_model->gravity = gravity;
        Eigen::VectorXd TauGrav = Eigen::VectorXd::Zero(rbdl_model->q_size);
        Eigen::VectorXd QDDotGrav = Eigen::VectorXd::Zero(rbdl_model->q_size);
        RigidBodyDynamics::InverseDynamics (*rbdl_model, robot_states.pos, robot_states.vel, QDDotGrav, TauGrav);
        // Get gravity and Coriolis forces
        return TauGrav;
    }
    Eigen::VectorXd Hand::getGravityComp() {
        rbdl_model->gravity = gravity;
        Eigen::VectorXd QDotGrav = Eigen::VectorXd::Zero(rbdl_model->q_size);
        Eigen::VectorXd TauGrav = Eigen::VectorXd::Zero(rbdl_model->q_size);
        Eigen::VectorXd QDDotGrav = Eigen::VectorXd::Zero(rbdl_model->q_size);

        RigidBodyDynamics::InverseDynamics (*rbdl_model, robot_states.pos, QDotGrav, QDDotGrav, TauGrav);
        // Get gravity and Coriolis forces
        return TauGrav;
    }

    size_t Hand::getNBfingers(){
        return _fingersNum;
    }

    //===============  Gains and Params ====================//
    // 
    void Hand::rotateGravity(const Eigen::Matrix3d& baseRotation){
        Eigen::Vector3d _gravity = Eigen::Vector3d(0., 0., -9.8);
        _gravity = baseRotation.transpose() * _gravity;
        setGravity(_gravity);
    }

    //==================     Output   =====================//
    //
    Eigen::Vector3d Hand::getFingerPos(size_t i){
        return finger[i].task_states.pos;
    }
    Eigen::Vector4d Hand::getFingerJointPos(size_t i){
        return finger[i].joint_states.pos;
    }

    Eigen::Vector4d Hand::getFingerQuat(size_t i){
        Eigen::Vector4d tt_orintation = Utils<double>::rotationMatrixToQuaternion(finger[i].rotMat);
        return tt_orintation;
    }

    Eigen::Vector3d Hand::getFingerAngVel(size_t i){
        return finger[i].task_states.angVel;
    }

    Eigen::Matrix3d Hand::getFingerRotMat(size_t i){
        return finger[i].rotMat;
    }

    Eigen::MatrixXd Hand::getFingerFullJacobian(size_t i){
        return finger[i].full_Jacobian;
    }


    Eigen::Vector3d Hand::getFingerVel(size_t i){
        return finger[i].task_states.vel;
    }
    Eigen::MatrixXd Hand::getFingerJacob(size_t i){
        return finger[i].jacobian;
    }
    Eigen::MatrixXd Hand::getFingerJacobQ(size_t i){
        return finger[i].full_Jacobian.block(0,4*i,3,4);
    }

    Eigen::MatrixXd Hand::getJacob(){
        Eigen::MatrixXd Jacobs = Eigen::MatrixXd::Zero(_fingersNum*3,_eachFingerDof*_fingersNum);
            for (size_t i = 0; i < _fingersNum; i++)
                Jacobs.block(3*i,4*i,3,4) = finger[i].jacobian;
        return Jacobs;
    }
    Eigen::MatrixXd Hand::getJacobDerivative(){
        Eigen::VectorXd jdot_qdot = Eigen::VectorXd::Zero(_fingersNum*3);
            for (size_t i = 0; i < _fingersNum*3; i++){
                jdot_qdot.segment(3*i,3) = getContactHessians(i).transpose();
            }
        return jdot_qdot;
    }


    Eigen::VectorXd Hand::getFingerInvKinematic(size_t i, const Eigen::VectorXd& pos){
        Eigen::Vector3d local_point (0., 0., 0.);

        Eigen::VectorXd Q = robot_states.pos;

        RigidBodyDynamics::UpdateKinematicsCustom (*rbdl_model, &Q, NULL, NULL);

        RigidBodyDynamics::InverseKinematicsConstraintSet cs;
        cs.AddPointConstraint (finger[i].ee_id , local_point, pos);
        // cs.AddPointConstraint (body_id_1 , local_point, target_pos1);
        // cs.AddPointConstraint (body_id_2 , local_point, target_pos2);
        // cs.AddPointConstraint (body_id_3 , local_point, target_pos3);
        Eigen::VectorXd Qinv (Q);
        bool result = RigidBodyDynamics::InverseKinematics (*rbdl_model, Q, cs, Qinv);
        // std::cout <<"the results are: " << result << std::endl;
        if(result){
            return Qinv.segment(4*i,4);
        }else{
            return Q.segment(4*i,4);
        }
    
    }

    Eigen::VectorXd Hand::getContactHessians(size_t f_indx, const Eigen::Vector3d& relPoint){
        Eigen::VectorXd qddot = Eigen::VectorXd::Zero(rbdl_model->q_size);
        Eigen::VectorXd tempAcc; tempAcc.resize(6);
        tempAcc = RigidBodyDynamics::CalcPointAcceleration6D(*rbdl_model, robot_states.pos, robot_states.vel,qddot, finger[f_indx].ee_id,relPoint,false);
        //! we just need the linear part here
        return tempAcc;
    }
    //==============================================//

    void Hand::updateKinematicModel(){
         RigidBodyDynamics::UpdateKinematics(*rbdl_model, robot_states.pos, robot_states.vel, robot_states.acc);
        for (size_t i = 0; i < _fingersNum; i++)
        {
            perform_fk(i);
        }
    }

    void Hand::perform_fk(size_t f_indx,const Eigen::Vector3d& relPoint){
        // std::cout << "ERROR 1" << std::endl;
        Eigen::Matrix3d worldToEe  = RigidBodyDynamics::CalcBodyWorldOrientation(*rbdl_model, robot_states.pos, finger[f_indx].ee_id,false);
        Eigen::VectorXd tempVel; tempVel.resize(6);
        tempVel = RigidBodyDynamics::CalcPointVelocity6D(*rbdl_model, robot_states.pos, robot_states.vel, finger[f_indx].ee_id,relPoint,false);
        // !this may be used later
        // Eigen::Vector3d eeAngVel = tempVel.head(3);
        
        Eigen::VectorXd tempAcc; tempAcc.resize(6);
        tempAcc = RigidBodyDynamics::CalcPointAcceleration6D(*rbdl_model, robot_states.pos, robot_states.vel,robot_states.acc, finger[f_indx].ee_id,relPoint,false);
        // !this may be used later 
        // Eigen::Vector3d eeAngAcc = tempAcc.head(3);
        
        finger[f_indx].task_states.pos = RigidBodyDynamics::CalcBodyToBaseCoordinates(*rbdl_model, robot_states.pos, finger[f_indx].ee_id,relPoint,false);
        finger[f_indx].task_states.vel  = tempVel.tail(3);
        finger[f_indx].task_states.angVel = tempVel.head(3);
        // finger[f_indx].task_states.acc  = tempAcc.tail(3); 
        finger[f_indx].rotMat = worldToEe.transpose();
       
        Eigen::MatrixXd Jacob =  Eigen::MatrixXd::Zero(6,rbdl_model->q_size);
        RigidBodyDynamics::CalcPointJacobian6D(*rbdl_model, robot_states.pos, finger[f_indx].ee_id, relPoint, Jacob, false);
        // std::cout << "jacob shape: " << Jacob.rows() << " x " << Jacob.cols() << std::endl;
        finger[f_indx].full_Jacobian = Jacob.block(0, 4*f_indx, 6, 4);
        // std::cout << "finger JACOBIAN :" <<finger[f_indx].full_Jacobian << std::endl << std::endl; 
    //    std::cout << "JACOBIAN :" <<Jacob << std::endl << std::endl; 
        
        finger[f_indx].jacobian = Jacob.block(3, 4*f_indx, 3, 4);
    }

    Eigen::VectorXd Hand::perform_id(){
        Eigen::VectorXd Tau = Eigen::VectorXd::Zero(rbdl_model->q_size);
        rbdl_model->gravity = gravity;
        RigidBodyDynamics::InverseDynamics(*rbdl_model, robot_states.pos, robot_states.vel, robot_states.acc, Tau);
        // Get gravity and Coriolis forces
        return Tau;
    }

    Eigen::VectorXd Hand::perform_fdyn(Eigen::VectorXd Tau){
        Eigen::VectorXd qdd = Eigen::VectorXd::Zero(rbdl_model->q_size);
        RigidBodyDynamics::ForwardDynamics (*rbdl_model, robot_states.pos, robot_states.vel, Tau, qdd);
        return qdd;
    }

}
