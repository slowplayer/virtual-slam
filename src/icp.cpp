#include "IcpSolver.h"
#include "PointCloudGenerator.h"

#include <iostream>

int main()
{
  PointCloudGenerator generator;
  IcpSolver icpSolver;
  
  Eigen::Matrix3d R_true,R_est;
  Eigen::Vector3d t_true,t_est;
  
  generator.generatePose(R_true,t_true);
  
  int n=10;
  for(int i=0;i<n;i++)
  {
    Eigen::Vector3d point3d1,point3d2;
    
    generator.generatePoint(point3d1);
    
    generator.transformWithNoise(R_true,t_true,point3d1,point3d2);
    
    icpSolver.add(point3d1,point3d2);
  }
  
  icpSolver.getTransformation(R_est,t_est);
  
  return 0;
}