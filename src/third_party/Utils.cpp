#include "thirdparty/Utils.h"

template<typename T>
Utils<T>::Utils()
{

}


template<typename T>
Eigen::Matrix<T,4,1> Utils<T>::quaternionProduct(Eigen::Matrix<T,4,1> q1, Eigen::Matrix<T,4,1> q2)
{
  Eigen::Matrix<T,4,1> q;
  q(0) = q1(0)*q2(0)-(q1.segment(1,3)).dot(q2.segment(1,3));
  Eigen::Matrix<T,3,1> q1Im = (q1.segment(1,3));
  Eigen::Matrix<T,3,1> q2Im = (q2.segment(1,3));
  q.segment(1,3) = q1(0)*q2Im+q2(0)*q1Im+q1Im.cross(q2Im);

  return q;
}


template<typename T>
Eigen::Matrix<T,3,3> Utils<T>::getSkewSymmetricMatrix(Eigen::Matrix<T,3,1> input)
{
  Eigen::Matrix<T,3,3> output;

  output << 0.0f, -input(2), input(1),
            input(2), 0.0f, -input(0),
            -input(1), input(0), 0.0f;

  return output;
}


template<typename T>
Eigen::Matrix<T,3,3> Utils<T>::eulerAnglesToRotationMatrix(T phi, T theta, T psi)
{
  T cphi = std::cos(phi);
  T sphi = std::sin(phi);
  T ctheta = std::cos(theta);
  T stheta = std::sin(theta);
  T cpsi = std::cos(psi);
  T spsi = std::sin(psi);


  Eigen::Matrix<T,3,3> R;
  R << cpsi*ctheta, cpsi*stheta*sphi-spsi*cphi, cpsi*stheta*cphi+spsi*sphi,
       spsi*ctheta, spsi*stheta*sphi+cpsi*cphi, spsi*stheta*cphi-cpsi*sphi,
       -stheta, ctheta*sphi, ctheta*cphi;

  return R;
}


template<typename T>
Eigen::Matrix<T,4,1> Utils<T>::rotationMatrixToQuaternion(Eigen::Matrix<T,3,3> R)
{
  Eigen::Matrix<T,4,1> q;

  float r11 = R(0,0);
  float r12 = R(0,1);
  float r13 = R(0,2);
  float r21 = R(1,0);
  float r22 = R(1,1);
  float r23 = R(1,2);
  float r31 = R(2,0);
  float r32 = R(2,1);
  float r33 = R(2,2);

  float tr = r11+r22+r33;
  float tr1 = r11-r22-r33;
  float tr2 = -r11+r22-r33;
  float tr3 = -r11-r22+r33;

  if(tr>0)
  {  
    q(0) = sqrt(1.0f+tr)/2.0f;
    q(1) = (r32-r23)/(4.0f*q(0));
    q(2) = (r13-r31)/(4.0f*q(0));
    q(3) = (r21-r12)/(4.0f*q(0));
  }
  else if((tr1>tr2) && (tr1>tr3))
  {
    q(1) = sqrt(1.0f+tr1)/2.0f;
    q(0) = (r32-r23)/(4.0f*q(1));
    q(2) = (r21+r12)/(4.0f*q(1));
    q(3) = (r31+r13)/(4.0f*q(1));
  }     
  else if((tr2>tr1) && (tr2>tr3))
  {   
    q(2) = sqrt(1.0f+tr2)/2.0f;
    q(0) = (r13-r31)/(4.0f*q(2));
    q(1) = (r21+r12)/(4.0f*q(2));
    q(3) = (r32+r23)/(4.0f*q(2));
  }
  else
  {
    q(3) = sqrt(1.0f+tr3)/2.0f;
    q(0) = (r21-r12)/(4.0f*q(3));
    q(1) = (r31+r13)/(4.0f*q(3));
    q(2) = (r32+r23)/(4.0f*q(3));        
  }

  return q;
}


template<typename T>
Eigen::Matrix<T,3,3> Utils<T>::quaternionToRotationMatrix(Eigen::Matrix<T,4,1> q)
{
  Eigen::Matrix<T,3,3> R;

  T q0 = q(0);
  T q1 = q(1);
  T q2 = q(2);
  T q3 = q(3);

  R(0,0) = q0*q0+q1*q1-q2*q2-q3*q3;
  R(1,0) = 2.0f*(q1*q2+q0*q3);
  R(2,0) = 2.0f*(q1*q3-q0*q2);

  R(0,1) = 2.0f*(q1*q2-q0*q3);
  R(1,1) = q0*q0-q1*q1+q2*q2-q3*q3;
  R(2,1) = 2.0f*(q2*q3+q0*q1);

  R(0,2) = 2.0f*(q1*q3+q0*q2);
  R(1,2) = 2.0f*(q2*q3-q0*q1);
  R(2,2) = q0*q0-q1*q1-q2*q2+q3*q3;  

  return R;
}


template<typename T>
void Utils<T>::quaternionToAxisAngle(Eigen::Matrix<T,4,1> q, Eigen::Matrix<T,3,1> &axis, T &angle)
{
  if((q.segment(1,3)).norm() < 1e-3f)
  {
    axis = q.segment(1,3);
  }
  else
  {
    axis = q.segment(1,3)/(q.segment(1,3)).norm();
    
  }

  angle = 2*std::acos(q(0));
}

template<typename T>
Eigen::Matrix<T,4,1> Utils<T>::axisAngleToQuaterion(Eigen::Matrix<T,3,1> axis, T angle)
{
  Eigen::Matrix<T,4,1> q;
  q(0) = std::cos(angle/2);
  q(1) = axis(0)*std::sin(angle/2);
  q(2) = axis(1)*std::sin(angle/2);
  q(3) = axis(2)*std::sin(angle/2);
  return q;
}


template<typename T>
Eigen::Matrix<T,4,1> Utils<T>::slerpQuaternion(Eigen::Matrix<T,4,1> q1, Eigen::Matrix<T,4,1> q2, T t)
{

  Eigen::Matrix<T,4,1> q;

  // Change sign of q2 if dot product of the two quaterion is negative => allows to interpolate along the shortest path
  if(q1.dot(q2)<0.0f)
  {   
    q2 = -q2;
  }

  T dotProduct = q1.dot(q2);
  if(dotProduct > 1.0f)
  {
    dotProduct = 1.0f;
  }
  else if(dotProduct < -1.0f)
  {
    dotProduct = -1.0f;
  }

  T omega = acos(dotProduct);

  if(std::fabs(omega)<FLT_EPSILON)
  {
    q = q1.transpose()+t*(q2-q1).transpose();
  }
  else
  {
    q = (std::sin((1-t)*omega)*q1+std::sin(t*omega)*q2)/std::sin(omega);
  }

  return q;
}

template<typename T>
Eigen::Matrix<T,4,1> Utils<T>::slerpQuaternion(Eigen::Matrix<T,4,1>* q, Eigen::Matrix<T,Eigen::Dynamic,1> t, int size)
{

  if(size==1)
  {
    return q[0];
  }
  else
  {
    T sum = 0.0f;
    for(int k = 0; k <size; k++)
    {
      sum+=t(k);
    }
    if(sum<FLT_EPSILON)
    {
      return slerpQuaternion(slerpQuaternion(q,t,size-1),q[size-1],0.0f);
    }
    else
    {
      return slerpQuaternion(slerpQuaternion(q,t,size-1),q[size-1],t(size-1)/sum);
    }
  }
}


template<typename T>
Eigen::Matrix<T,3,3> Utils<T>::rodriguesRotation(Eigen::Matrix<T,3,1> v1, Eigen::Matrix<T,3,1> v2)
{
  // Compute rotation error between current orientation and plane orientation using Rodrigues' law
  v1.normalize();
  v2.normalize();

  Eigen::Matrix<T,3,1> w;
  w = v1.cross(v2);
  float c = v1.dot(v2);  
  float s = w.norm();
  w /= s;
  
  Eigen::Matrix<T,3,3> K;
  K << getSkewSymmetricMatrix(w);

  Eigen::Matrix<T,3,3> Re;
  if(fabs(s)< FLT_EPSILON)
  {
    Re = Eigen::Matrix<T,3,3>::Identity();
  }
  else
  {
    Re = Eigen::Matrix<T,3,3>::Identity()+s*K+(1-c)*K*K;
  }

  return Re;
}

template<typename T>
Eigen::Matrix<T,3,1> Utils<T>::quaternionToAngularVelocity(Eigen::Matrix<T,4,1> q1, Eigen::Matrix<T,4,1> q2, T gain)
{
  Eigen::Matrix<T,4,1> q1I, wq;
  q1I(0) = q1(0);
  q1I.segment(1,3) = -q1.segment(1,3);
  wq = 2.0f*gain*quaternionProduct(q2-q1,q1I);
  
  return wq.segment(1,3);
}


template<typename T>
Eigen::Matrix<T,3,3> Utils<T>::orthogonalProjector(Eigen::Matrix<T,3,1> v)
{ 
  return Eigen::Matrix<T,3,3>::Identity()-v*v.transpose();
}


template<typename T>
T Utils<T>::smoothRise(T x, T a, T b)
{
  T y; 
  if(x<a)
  {
    y = 0.0f;
  }
  else if(x>b)
  {
    y = 1.0f;
  }
  else
  {
    y = (1.0f+sin(M_PI*(x-a)/(b-a)-M_PI/2.0f))/2.0f;
  }

  return y;
}


template<typename T>
T Utils<T>::smoothFall(T x, T a, T b)
{
  return 1.0f-smoothRise(x,a,b);
}


template<typename T>
T Utils<T>::smoothRiseFall(T x, T a, T b, T c, T d)
{
  return smoothRise(x,a,b)*smoothFall(x,c,d);
}


template<typename T>
T Utils<T>::deadZone(T x, T a, T b)
{
  if(x < b && x > a)
  {
    return 0.0f;
  }
  else
  {
    return x;
  }
}


template<typename T>
Eigen::Matrix<T,Eigen::Dynamic,1> Utils<T>::deadZone(Eigen::Matrix<T,Eigen::Dynamic,1> x, T limit)
{
  T norm = x.norm();

  if(norm>limit)
  {
    return x;
  }
  else
  {
    return Eigen::Matrix<T,Eigen::Dynamic,1>::Zero(x.size());
  }
}


template<typename T>
T Utils<T>::wrapToZero(T x, T a, T b)
{
  if(x < b && x > a)
  {
    return x;
  }
  else
  {
    return 0.0f;
  }
}


template<typename T>
T Utils<T>::bound(T x, T a, T b)
{
  if(x > b)
  {
    return b;
  }
  else if(x<a)
  {
    return a;
  }
  else
  {
    return x;
  }
}


template<typename T>
Eigen::Matrix<T,Eigen::Dynamic,1> Utils<T>::bound(Eigen::Matrix<T,Eigen::Dynamic,1> x, T limit)
{
  T norm = x.norm();

  if(norm>limit)
  {
    return x*limit/norm;
  }
  else
  {
    return x;
  }
}


template<typename T>
Eigen::Matrix<T,4,4> Utils<T>::getDHMatrix(T a, T alpha, T d, T theta, DH_CONVENTION dhConvention)
{
  Eigen::Matrix<T,4,4> H;

  if(dhConvention == NORMAL)
  {
    H(0,0) = std::cos(theta);
    H(0,1) = -std::cos(alpha)*std::sin(theta);
    H(0,2) = std::sin(alpha)*std::sin(theta);
    H(0,3) = a*std::cos(theta);

    H(1,0) = std::sin(theta);
    H(1,1) = std::cos(alpha)*std::cos(theta);
    H(1,2) = -std::sin(alpha)*std::cos(theta);
    H(1,3) = a*std::sin(theta);

    H(2,0) = 0.0f;
    H(2,1) = std::sin(alpha);
    H(2,2) = std::cos(alpha);
    H(2,3) = d;

    H(3,0) = 0.0f;
    H(3,1) = 0.0f;
    H(3,2) = 0.0f;
    H(3,3) = 1.0f;
  }
  else
  {
    H(0,0) = std::cos(theta);
    H(0,1) = -std::sin(theta);
    H(0,2) = 0.0f;
    H(0,3) = a;

    H(1,0) = std::sin(theta)*std::cos(alpha);
    H(1,1) = std::cos(theta)*std::cos(alpha);
    H(1,2) = -std::sin(alpha);
    H(1,3) = -d*std::sin(alpha);

    H(2,0) = std::sin(theta)*std::sin(alpha);
    H(2,1) = std::cos(theta)*std::sin(alpha);
    H(2,2) = std::cos(alpha);
    H(2,3) = d*std::cos(alpha);

    H(3,0) = 0.0f;
    H(3,1) = 0.0f;
    H(3,2) = 0.0f;
    H(3,3) = 1.0f;
  }

  return H;
}


template<typename T>
Eigen::Matrix<T,4,4> Utils<T>::getForwardKinematics(Eigen::Matrix<T,7,1> joints, ROBOT_ID robotID)
{
  Eigen::Matrix<T,4,4> H, H1, H2, H3, H4, H5, H6, H7, H8;

  if(robotID==KUKA_LWR)
  {
    H1 = getDHMatrix(0.0f,M_PI/2.0f,0.3105f,joints(0));
    H2 = getDHMatrix(0.0f,-M_PI/2.0f,0.0f,joints(1));
    H3 = getDHMatrix(0.0f,-M_PI/2.0f,0.4f,joints(2));
    H4 = getDHMatrix(0.0f,M_PI/2.0f,0.0f,joints(3));
    H5 = getDHMatrix(0.0f,M_PI/2.0f,0.39f,joints(4));
    H6 = getDHMatrix(0.0f,-M_PI/2.0f,0.0f,joints(5));
    H7 = getDHMatrix(0.0f,0.0f,0.078f,joints(6));    
    H = H1*H2*H3*H4*H5*H6*H7;
  }
  else if(robotID==FRANKA_PANDA)
  {
    H1 = getDHMatrix(0.0f,0.0f,0.333f,joints(0),MODIFIED);
    H2 = getDHMatrix(0.0f,-M_PI/2.0f,0.0f,joints(1),MODIFIED);
    H3 = getDHMatrix(0.0f,M_PI/2.0f,0.316f,joints(2),MODIFIED);
    H4 = getDHMatrix(0.0825f,M_PI/2.0f,0.0f,joints(3),MODIFIED);
    H5 = getDHMatrix(-0.0825f,-M_PI/2.0f,0.384f,joints(4),MODIFIED);
    H6 = getDHMatrix(0.0f,M_PI/2.0f,0.0f,joints(5),MODIFIED);
    H7 = getDHMatrix(0.088f,M_PI/2.0f,0.0f,joints(6),MODIFIED);  
    H8 = getDHMatrix(0.0f,0.0f,0.107f,0.0f,MODIFIED); 
    H = H1*H2*H3*H4*H5*H6*H7*H8;
  }

  return H;
}


template<typename T>
Eigen::Matrix<T,6,7> Utils<T>::getGeometricJacobian(Eigen::Matrix<T,7,1> joints, Eigen::Matrix<T,3,1> rEEx, ROBOT_ID robotID)
{
  Eigen::Matrix<T,4,4> Hee, H[8], Hk;
  Eigen::Matrix<T,6,7> J;

  DH_CONVENTION convention;
  if(robotID == KUKA_LWR)
  {
    convention = NORMAL;
  }
  else if(robotID = FRANKA_PANDA)
  {
    convention = MODIFIED;
  }

  if(robotID == KUKA_LWR)
  {  
    H[0] = getDHMatrix(0.0f,M_PI/2.0f,0.3105f,joints(0));
    H[1] = getDHMatrix(0.0f,-M_PI/2.0f,0.0f,joints(1));
    H[2] = getDHMatrix(0.0f,-M_PI/2.0f,0.4f,joints(2));
    H[3] = getDHMatrix(0.0f,M_PI/2.0f,0.0f,joints(3));
    H[4] = getDHMatrix(0.0f,M_PI/2.0f,0.39f,joints(4));
    H[5] = getDHMatrix(0.0f,-M_PI/2.0f,0.0f,joints(5));
    H[6] = getDHMatrix(0.0f,0.0f,0.078f,joints(6));
    Hee = H[0]*H[1]*H[2]*H[3]*H[4]*H[5]*H[6];
  }
  else if(robotID == FRANKA_PANDA)
  {
    H[0] = getDHMatrix(0.0f,0.0f,0.333f,joints(0),convention);
    H[1] = getDHMatrix(0.0f,-M_PI/2.0f,0.0f,joints(1),convention);
    H[2] = getDHMatrix(0.0f,M_PI/2.0f,0.316f,joints(2),convention);
    H[3] = getDHMatrix(0.0825f,M_PI/2.0f,0.0f,joints(3),convention);
    H[4] = getDHMatrix(-0.0825f,-M_PI/2.0f,0.384f,joints(4),convention);
    H[5] = getDHMatrix(0.0f,M_PI/2.0f,0.0f,joints(5),convention);
    H[6] = getDHMatrix(0.088f,M_PI/2.0f,0.107f,joints(6),convention);  
    // H[7] = getDHMatrix(0.0f,0.0f,0.107f,0.0f,MODIFIED); 
    Hee = H[0]*H[1]*H[2]*H[3]*H[4]*H[5]*H[6];
  }

  // std::cerr << H[0] << std::endl << std::endl;
  // std::cerr << H[0]*H[1] << std::endl << std::endl;
  // std::cerr << H[0]*H[1]*H[2] << std::endl << std::endl;
  // std::cerr << H[0]*H[1]*H[2]*H[3] << std::endl << std::endl;
  // std::cerr << H[0]*H[1]*H[2]*H[3]*H[4] << std::endl << std::endl;
  // std::cerr << H[0]*H[1]*H[2]*H[3]*H[4]*H[5] << std::endl << std::endl;
  // std::cerr << H[0]*H[1]*H[2]*H[3]*H[4]*H[5]*H[6] << std::endl << std::endl;

  Eigen::Matrix<T,3,1> xEE, z0, x0, xk, zk;

  xEE = Hee.block(0,3,3,1);



  Hk.setIdentity();

  J.setConstant(0.0f);



  for(int k = 0; k < 7; k++)
  {

    if(convention==MODIFIED)
    {
      Hk = Hk*H[k];
    }
    xk = Hk.block(0,3,3,1);
    zk = Hk.block(0,2,3,1);

    J.block(0,k,3,1) = zk.cross(xEE-xk);
    J.block(3,k,3,1) = zk;
    if(convention == NORMAL)
    {
      Hk = Hk*H[k];
    }
  }

  J.block(0,0,3,7) += -getSkewSymmetricMatrix(rEEx)*J.block(3,0,3,7); 

  return J;

}


template class Utils<float>;
template class Utils<double>;
