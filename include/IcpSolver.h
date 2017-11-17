#ifndef ICP_SOLVER_H
#define ICP_SOLVER_H

#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Sparse>

class IcpSolver
{
public:
  IcpSolver();
  ~IcpSolver(){};
  
  
  inline double getAccumulatedWeight()const {return accumulated_weight_;}
  inline unsigned int getNoOfSamples(){return no_of_samples_;}
  
  void reset();
  void add(const Eigen::Vector3d& point,const Eigen::Vector3d& corresponding_point);
  void getTransformation(Eigen::Matrix3d &R,Eigen::Vector3d &t);
  
protected:
  unsigned int no_of_samples_;
  double accumulated_weight_;
  Eigen::Vector3d mean1_,mean2_;
  Eigen::Matrix3d covariance_;
};
#endif