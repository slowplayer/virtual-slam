#include "IcpSolver.h"

IcpSolver::IcpSolver():
  no_of_samples_(0),accumulated_weight_(0),
  mean1_(Eigen::Vector3d::Identity()),
  mean2_(Eigen::Vector3d::Identity()),
  covariance_(Eigen::Matrix3d::Identity())
{
  reset();
}
void IcpSolver::reset()
{
  no_of_samples_=0;
  accumulated_weight_=0.0;
  mean1_.fill(0);
  mean2_.fill(0);
  covariance_.fill(0);
}
void IcpSolver::add(const Eigen::Vector3d& point, const Eigen::Vector3d& corresponding_point)
{
  
  double weight=1.0/(point(2)*corresponding_point(2));
  ++no_of_samples_;
  accumulated_weight_+=weight;
  double alpha=weight/accumulated_weight_;
  
  Eigen::Vector3d diff1,diff2;
  //no centroid point
  diff1=point-mean1_;
  diff2=point-mean2_;
  
  covariance_=(1.0f-alpha)*(covariance_+alpha*(diff2*diff1.transpose()));
  
  //centroid point 
  mean1_+=alpha*diff1;
  mean2_+=alpha*diff2;
}
void IcpSolver::getTransformation(Eigen::Matrix3d& R, Eigen::Vector3d& t)
{
  //A=U*A*Vt
  Eigen::JacobiSVD<Eigen::Matrix<double,3,3> > svd(covariance_,Eigen::ComputeFullU|Eigen::ComputeFullV);
  const Eigen::Matrix<double,3,3>& u=svd.matrixU();
  const Eigen::Matrix<double,3,3>& v=svd.matrixV();
  
  Eigen::Matrix3d s;
  s.setIdentity();
  if(u.determinant()*v.determinant()<0.0f)
    s(2,2)=-1.0f;
  
  //R=U*Vt
  R=u*s*v.transpose();
  //t=p-R*p'
  t=mean2_-R*mean1_;
}
