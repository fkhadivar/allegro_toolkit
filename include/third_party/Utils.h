#ifndef __UTILS_H__
#define __UTILS_H__

#include "ros/ros.h"
#include "Eigen/Eigen"

template<typename T = float>
class Utils 
{
	public:

	enum ROBOT_ID {KUKA_LWR, FRANKA_PANDA};


	enum DH_CONVENTION {NORMAL, MODIFIED};

	// Class constructor
	Utils();

    static Eigen::Matrix<T,4,1> quaternionProduct(Eigen::Matrix<T,4,1> q1, Eigen::Matrix<T,4,1> q2);

    static Eigen::Matrix<T,3,3> getSkewSymmetricMatrix(Eigen::Matrix<T,3,1> input);

    static Eigen::Matrix<T,3,3> eulerAnglesToRotationMatrix(T phi, T theta, T psi);

    static Eigen::Matrix<T,4,1> rotationMatrixToQuaternion(Eigen::Matrix<T,3,3> R);

  	static Eigen::Matrix<T,3,3> quaternionToRotationMatrix(Eigen::Matrix<T,4,1> q);

	static void quaternionToAxisAngle(Eigen::Matrix<T,4,1> q, Eigen::Matrix<T,3,1> &axis, T &angle);

	static Eigen::Matrix<T,4,1> axisAngleToQuaterion(Eigen::Matrix<T,3,1> axis, T angle);

  	static Eigen::Matrix<T,4,1> slerpQuaternion(Eigen::Matrix<T,4,1> q1, Eigen::Matrix<T,4,1> q2, T t);

	static Eigen::Matrix<T,4,1> slerpQuaternion(Eigen::Matrix<T,4,1>* q, Eigen::Matrix<T,Eigen::Dynamic,1> t, int size);

	static Eigen::Matrix<T,3,3> rodriguesRotation(Eigen::Matrix<T,3,1> v1, Eigen::Matrix<T,3,1> v2);

	static Eigen::Matrix<T,3,1> quaternionToAngularVelocity(Eigen::Matrix<T,4,1> q1, Eigen::Matrix<T,4,1> q2, T gain = 1.0f);

	static Eigen::Matrix<T,3,3> orthogonalProjector(Eigen::Matrix<T,3,1> v);

  	static T smoothRise(T x, T a, T b);

	static T smoothFall(T x, T a, T b);

	static T smoothRiseFall(T x, T a, T b, T c, T d);

	static T deadZone(T x, T a, T b);

	static Eigen::Matrix<T,Eigen::Dynamic,1> deadZone(Eigen::Matrix<T,Eigen::Dynamic,1> x, T limit);


	static T wrapToZero(T x, T a, T b);	

	static T bound(T x, T a, T b);

	static Eigen::Matrix<T,Eigen::Dynamic,1> bound(Eigen::Matrix<T,Eigen::Dynamic,1> x, T limit);

	static Eigen::Matrix<T,4,4> getDHMatrix(T a, T alpha, T d, T theta, DH_CONVENTION dhConvention = NORMAL);

	static Eigen::Matrix<T,4,4> getForwardKinematics(Eigen::Matrix<T,7,1> joints, ROBOT_ID robotID = KUKA_LWR);


	static Eigen::Matrix<T,6,7> getGeometricJacobian(Eigen::Matrix<T,7,1> joints, Eigen::Matrix<T,3,1> rEEx = Eigen::Matrix<T,3,1>::Zero(), ROBOT_ID roobtID = KUKA_LWR);

};

#endif