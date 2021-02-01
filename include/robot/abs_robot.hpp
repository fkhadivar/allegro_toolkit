#ifndef __ABSTRACT_ROBOT__
#define __ABSTRACT_ROBOT__
#include <Eigen/Dense>

namespace robot
{   
    struct States
    {
        Eigen::VectorXd pos, vel, acc, trq, angVel;
    };

    class AbsRobot
    {
        /* data */
    public:
        AbsRobot(){}
        virtual ~AbsRobot(){}
        //** Update Functions
        virtual void updateStates(const States& _states){
            robot_states = _states;
        }
        virtual void updateJntPos(const Eigen::VectorXd& pos){
            checkDim(pos);
            robot_states.pos = pos;
        }
        virtual void updateJntVel(const Eigen::VectorXd& vel){
            checkDim(vel);
            robot_states.pos = vel;
        }
        virtual void updateJntTrq(const Eigen::VectorXd& trq){
            checkDim(trq);
            robot_states.trq = trq;
        }
        //** Set Functions
        void setDof(size_t _dof){
            dof = _dof;
            robot_states.pos = Eigen::VectorXd::Zero(dof);
            robot_states.vel = Eigen::VectorXd::Zero(dof);
            robot_states.acc = Eigen::VectorXd::Zero(dof);
            robot_states.trq = Eigen::VectorXd::Zero(dof);
        }

        void setGravity(const Eigen::Vector3d& _gravity){
            gravity = _gravity;
        }
        //** Get Functions
        virtual States getStates() const{
            return robot_states;
        }
        size_t getDof() const {
            return dof;
        }

        Eigen::Vector3d getGravity(){
            return gravity;
        }
        virtual Eigen::VectorXd getGravityComp() = 0;
        //todo :
        //! getting and setting force limits

    protected:
        virtual void updateKinematicModel() = 0;

        void checkDim(Eigen::VectorXd vec){
            if (vec.size() != dof)
                std::abort();
        }

    protected:
        States robot_states;
        size_t dof;
        Eigen::Vector3d gravity = Eigen::Vector3d(0., 0., -9.8);

    };

} // namespace robot










#endif