
//! Initialize the robot model

#include "control/ds_force_control.h"
#include <memory>



namespace control {
    namespace ds_force{


        DsForceControl::DsForceControl(const std::shared_ptr<robot::AbsRobot>& _robot, Params& _params):params(_params){
            addRobot(_robot);
            dsController = std::make_shared<DSController>(3,params.psvLambda, params.psvDissip*params.psvLambda);
            nullPosition =  Eigen::VectorXd::Zero(robots[0]->getDof()); // todo move this to the hand
        }
        void DsForceControl::setInput(){
        }
        void DsForceControl::setParams(Params& _params){
            params = _params;
            dsController->set_damping_eigval(params.psvLambda, params.psvDissip*params.psvLambda);
        }
        void DsForceControl::setDsGain(const double dsGain){
            params.dsGain = dsGain;
        }
        void DsForceControl::setPsvGain(const double psvLambda, const double psvDissip){
            params.psvLambda = psvLambda;
            params.psvDissip = psvDissip;
            dsController->set_damping_eigval(params.psvLambda, params.psvDissip*params.psvLambda);
        }
        void DsForceControl::setNullPos(const Eigen::VectorXd& nullPose){
            nullPosition = nullPose;
        }

        Eigen::VectorXd DsForceControl::getOutput() {
            algorithm();
            //todo std:pair 
            return cmdTorque;
        }

        Eigen::VectorXd DsForceControl::getPlotVariable(){
            return plotVariable;
        }

        void DsForceControl::updateHandBasePose(const Eigen::Matrix3d& ref2Base){
            /*
                        left hand:    #  #  #
                ref: ^                   #  #  #
                    z|                   #  #  # 
                    |              ############
                x @-------->y         #######    
            */
        //    _Ref2Base = ref2Base;
        //    _gravity  = _Ref2Base * _gravity;
        //    _baseQuat = Utils<double>::rotationMatrixToQuaternion(_Ref2Base);

        }


        void DsForceControl::algorithm(){
            cmdTorque = Eigen::VectorXd::Zero(robots[0]->getDof());

            //this params are those to get the most out of psv ds
            // controlParams.dsGain = 1.;
            // controlParams.dsMaxVel = .05;
            // controlParams.dsRbfGain = 15.;
            // controlParams.psvLambda = 5.;
            // controlParams.psvDissip = .5;
            // controlParams.nullGain = 1.;
            //********set desired task
            //todo move this part in a function
            std::vector<Eigen::Vector3d> targetPositions(4);
            targetPositions[0] =  Eigen::Vector3d(.05,-.05,.17);
            targetPositions[1] =  Eigen::Vector3d(.05,  0.,.17);
            targetPositions[2] =  Eigen::Vector3d(.05, .05,.17);
            targetPositions[3] =  Eigen::Vector3d(.05,-.05,.05);

            if (_step > 500 )
            {
                targetPositions[0] =  Eigen::Vector3d(.1,-.05,.08);
                targetPositions[1] =  Eigen::Vector3d(.1,  0.,.08);
                targetPositions[2] =  Eigen::Vector3d(.1, .05,.08);
                targetPositions[3] =  Eigen::Vector3d(.05, .0,.0);
            }
            // if (_step > 1000)
            // {
            //     targetPositions[0] =  Eigen::Vector3d(.1,-.07,.15);
            //     targetPositions[1] =  Eigen::Vector3d(.1,  0.,.15);
            //     targetPositions[2] =  Eigen::Vector3d(.1, .07,.15);
            //     targetPositions[3] =  Eigen::Vector3d(.05, -.05,.05);
            // }
            // if (_step > 1500)
            // {
            //     targetPositions[0] =  Eigen::Vector3d(.1,-.05,.08);
            //     targetPositions[1] =  Eigen::Vector3d(.1,  0.,.08);
            //     targetPositions[2] =  Eigen::Vector3d(.1, .05,.08);
            //     targetPositions[3] =  Eigen::Vector3d(.05, .0,.0);
            // }

            if (_step > 1000){
                for (size_t i = 0; i < 4 ; i++){
                    targetPositions[i] = std::static_pointer_cast<robot::Hand>(robots[0])->finger[i].task_states.pos;
                }
            }
            //*******get force from passie Ds
            size_t numfingers = std::static_pointer_cast<robot::Hand>(robots[0])->finger.size();
            for (size_t i = 0; i < numfingers ; i++){
                Eigen::Vector3d fing_pos = std::static_pointer_cast<robot::Hand>(robots[0])->finger[i].task_states.pos;
                //** get the Ds
            if (_step < 10)
                    targetPositions[i] = fing_pos;
                Eigen::Vector3d fing_vel = std::static_pointer_cast<robot::Hand>(robots[0])->finger[i].task_states.vel;
                Eigen::MatrixXd fing_jacob = std::static_pointer_cast<robot::Hand>(robots[0])->finger[i].jacobian;
                Eigen::VectorXd desDs = control::util::computeDsRBF(fing_pos, targetPositions[i],params.dsGain,params.dsRbfGain,params.dsMaxVel);
                dsController->Update(fing_vel, desDs);
                Eigen::Vector4d taskTorque = fing_jacob.transpose() * dsController->control_output();
            
                if (i==0)
                    plotVariable = fing_pos -  targetPositions[i];
                // std::cout << "task command :" <<std::endl << taskTorque.transpose() << std::endl << std::endl; 
                //**null control
                Eigen::Vector4d nullTorque = control::util::getNullTorque(i,params.nullGain,nullPosition.segment(4*i,4),std::static_pointer_cast<robot::Hand>(robots[0])->finger[i].joint_states.pos,fing_jacob);               

                //** sum up
                cmdTorque.segment(4*i,4) = taskTorque + nullTorque;
            }
            //********compute the gravity compent
            Eigen::Matrix3d baseRotMat;
            baseRotMat = Eigen::AngleAxisd(-1*M_PI,Eigen::Vector3d::UnitX());
            std::static_pointer_cast<robot::Hand>(robots[0])->rotateGravity(baseRotMat);
            Eigen::VectorXd grvComp = std::static_pointer_cast<robot::Hand>(robots[0])->getCoriolisAndGravityForces(); 
            cmdTorque +=  grvComp;

            double EPSILON_FORCE = 1e-3;
            Eigen::VectorXd trqLim = std::static_pointer_cast<robot::Hand>(robots[0])->getTrqLimits();

            for (size_t i = 0; i < cmdTorque.size(); i++){
                if (fabs(cmdTorque[i]) < EPSILON_FORCE )
                    cmdTorque[i] = EPSILON_FORCE;
                else if ( cmdTorque[i] >  trqLim[i] )
                    cmdTorque[i] = trqLim[i];
                else if ( cmdTorque[i] <  -trqLim[i])
                    cmdTorque[i] = -trqLim[i];
            }
            _step ++;
        }
    }
}

