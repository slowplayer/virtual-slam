#include "PnpSolver.h"
#include "PointCloudGenerator.h"

#include <iostream>

int main()
{
  PointCloudGenerator generator;
  PnpSolver pnpSolver;
  
  Eigen::Matrix3d R_true,R_est;
  Eigen::Vector3d t_true,t_est;
  
  generator.generatePose(R_true,t_true);
  
  int n=10;
  for(int i=0;i<n;i++)
  {
    Eigen::Vector3d point3d;
    Eigen::Vector2d point2d;
    
    generator.generatePoint(point3d);
    
    generator.projectWithNoise(R_true,t_true,point3d,point2d);
  
    pnpSolver.add_correspondence(point3d,point2d);
  }
  
  pnpSolver.compute_pose(R_est,t_est);
   
  return 0;
}