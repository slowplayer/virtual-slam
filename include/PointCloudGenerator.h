#ifndef POINT_ClOUD_GENERATOR_H
#define POINT_ClOUD_GENERATOR_H

#include <cstdlib>
#include <time.h>
#include <cmath>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

class PointCloudGenerator
{
public:
  PointCloudGenerator();
  ~PointCloudGenerator(){};
  
  void set_point_radius(double radius);
  void set_transform_radius(double radius);
  void set_internal_parameters(double uc,double vc,double fu,double fv);
  void set_project_noise(double noise);
  void set_transform_noise(double noise);
  
  void generatePose(Eigen::Matrix3d &R,Eigen::Vector3d &t);
  void generatePoint(Eigen::Vector3d &point);
  
  void projectWithNoise(const Eigen::Matrix3d &R,const Eigen::Vector3d &t,
			const Eigen::Vector3d &point3d,Eigen::Vector2d &point2d);
  void projectWithoutNoise(const Eigen::Matrix3d &R,const Eigen::Vector3d &t,
			const Eigen::Vector3d &point3d,Eigen::Vector2d &point2d);
  
  void transformWithNoise(const Eigen::Matrix3d &R,const Eigen::Vector3d &t,
			const Eigen::Vector3d &point3d1,Eigen::Vector3d &point3d2);
  void transformWithoutNoise(const Eigen::Matrix3d &R,const Eigen::Vector3d &t,
			const Eigen::Vector3d &point3d1,Eigen::Vector3d &point3d2);
private:  
  double randomNumber(double min,double max);
  
  double cx,cy,fx,fy;
  double project_noise,transform_noise;
  double transform_radius,point_radius;
};
#endif