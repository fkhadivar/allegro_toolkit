#include "control/control_util.h"
namespace control{
    namespace util{

        QP::QP(){
            _solver = std::make_shared<control::qp_solver::QPOases>();
        }
        QP::~QP(){};
        void QP::setup_qp_tasks(std::vector<Eigen::MatrixXd>& A_t, std::vector<Eigen::VectorXd>& b_t, std::vector<Eigen::VectorXd>& w_t){
            size_t _dim = A_t[0].cols(); 
            size_t num_of_tasks =0;
            for (size_t i = 0; i < A_t.size(); i++)
                num_of_tasks += A_t[i].rows();
            Eigen::MatrixXd A = Eigen::MatrixXd::Zero(num_of_tasks , _dim);
            Eigen::VectorXd b = Eigen::VectorXd::Zero(num_of_tasks);
            size_t t_index = 0;
            for (int i = 0; i < A_t.size(); i++){
                for (int j = 0; j < A_t[i].rows(); j++)
                    A_t[i].row(j).array() *= w_t[i](j);
                b_t[i].array() *= w_t[i].array();
                A.block(t_index, 0, A_t[i].rows(), A_t[i].cols()) = A_t[i];
                b.segment(t_index, b_t[i].size()) = b_t[i].transpose();
                t_index +=A_t[i].rows();
            }
            _H = A.transpose() * A;
            _g = -A.transpose() * b;
        }
        void QP::setup_qp_constraints(std::vector<Eigen::MatrixXd>& A_c, std::vector<Eigen::MatrixXd>& b_c){
            size_t _dim = A_c[0].cols(); 
            size_t num_of_constraints =0;
            for (size_t i = 0; i < A_c.size(); i++)
                num_of_constraints += A_c[i].rows();
            _A = Eigen::MatrixXd::Zero(num_of_constraints, _dim);
            _ubA = Eigen::VectorXd::Zero(num_of_constraints);
            _lbA = Eigen::VectorXd::Zero(num_of_constraints);
            size_t c_index = 0;
            for (int i = 0; i < A_c.size(); i++){
                _A.block(c_index, 0,  A_c[i].rows(),   A_c[i].cols()) = A_c[i];
                _lbA.segment(c_index, b_c[i].cols()) = b_c[i].row(0);
                _ubA.segment(c_index, b_c[i].cols()) = b_c[i].row(1);
                c_index +=A_c[i].rows();
            }
        }
        void QP::setup_qp_bounds(Eigen::VectorXd& lb, Eigen::VectorXd& ub){
            _lb = lb;
            _ub = ub;
        }

        bool QP::qp_solve(){
            if (_solver != nullptr)
                    return _solver->solve(_H, _g, _A, _lb, _ub, _lbA, _ubA);
            return false;
        }

        Eigen::VectorXd QP::qp_solution() const{
            if (_solver)
                return _solver->get_solution();
            return Eigen::VectorXd();
        }

        Eigen::VectorXd computeDs(const Eigen::VectorXd& pos, const Eigen::VectorXd& desPos,
                                        const double& dsGain, const double& maxDx){
                Eigen::VectorXd deltaX = desPos - pos;
                if (deltaX.norm() > maxDx)
                    deltaX = maxDx * deltaX.normalized();
                Eigen::VectorXd desVel = dsGain * deltaX;
                return desVel;
        }

        Eigen::VectorXd computeDsRBF(const Eigen::VectorXd& pos, const Eigen::VectorXd& desPos,
                                        const double& dsGain, const double& dsRbfGain, const double& maxDx, const double& lambda){
                Eigen::VectorXd deltaX = desPos - pos;
                if (deltaX.norm() > maxDx)
                    deltaX = maxDx * deltaX.normalized();
                
                Eigen::VectorXd rbf_vector =  Eigen::VectorXd::Zero(pos.size());
                rbf_vector.head(pos.size()) =  deltaX;
                Eigen::MatrixXd Sigma = (1/(4*maxDx*maxDx)) * Eigen::MatrixXd::Identity(3,3);
                double thetagain = -0.5 *  rbf_vector.transpose() * Sigma * rbf_vector;
                Eigen::VectorXd desVel = (1/lambda)*(dsGain + dsRbfGain * std::exp(thetagain)) * deltaX;
                return desVel;
        }
        double computeRbfGain(const Eigen::VectorXd& x, const Eigen::VectorXd& xd,
                                        const double& maxGain, const double& minGain, const double& maxDx){
                Eigen::VectorXd deltaX = xd - x;
                if (deltaX.norm() > maxDx)
                    deltaX = maxDx * deltaX.normalized();
                
                Eigen::VectorXd rbf_vector =  Eigen::VectorXd::Zero(x.size());
                rbf_vector.head(x.size()) =  deltaX;
                Eigen::MatrixXd Sigma = (1/(4*maxDx*maxDx)) * Eigen::MatrixXd::Identity(3,3);
                double thetagain = -0.5 *  rbf_vector.transpose() * Sigma * rbf_vector;
                return minGain + (maxGain-minGain) * std::exp(thetagain);
        }

        Eigen::VectorXd getNullTorque(const size_t& ind, const double& nullGain, const Eigen::Vector4d& nullpose,
                                    const Eigen::Vector4d& robpose, const Eigen::MatrixXd& fing_jacob){

            Eigen::VectorXd temNullTorque = Eigen::VectorXd::Zero(nullpose.size());
            for (size_t j = 0; j < nullpose.size(); j++){
                temNullTorque[j] = -.05 * (robpose[j]  - nullpose[j]);
            }
            Eigen::MatrixXd tempMat = fing_jacob * fing_jacob.transpose();
            Eigen::Matrix4d nullMat =  Eigen::MatrixXd::Identity(nullpose.size(),nullpose.size()) - fing_jacob.transpose() * tempMat.inverse() * fing_jacob;
            Eigen::Vector4d nullTorque = nullMat * temNullTorque;
            if (nullTorque[0] != nullTorque[0])
                nullTorque = Eigen::VectorXd::Zero(nullpose.size());
            return nullTorque;
        }

        double smoothVel(const Eigen::Vector3d& vel, const double& minGain,const double& sigma){
            double _gain = 1.;
            double Sig = 1. / (sigma*sigma);
            _gain = std::exp(-0.5 *Sig* vel.transpose() * vel);
            if(_gain < minGain)
                _gain = minGain;
            return _gain;
        }
    } // namespace util
} // namespace control
