#include "PointCloudGenerator.h"

PointCloudGenerator::PointCloudGenerator()
:fx(800),fy(800),cx(320),cy(240),transform_noise(1.0),
project_noise(1.0),transform_radius(1.0),point_radius(2.0)
{
  srand(time(0));
}
double PointCloudGenerator::randomNumber(double min, double max)
{
    return min+(max-min)*double(rand())/RAND_MAX;
}
void PointCloudGenerator::set_internal_parameters(double uc, double vc, double fu, double fv)
{
    cx=uc;
    cy=vc;
    fx=fu;
    fy=fv;
}
void PointCloudGenerator::set_project_noise(double noise)
{
  project_noise=noise;
}
void PointCloudGenerator::set_transform_noise(double noise)
{
  transform_noise=noise;
}
void PointCloudGenerator::set_point_radius(double radius)
{
  point_radius=radius;
}
void PointCloudGenerator::set_transform_radius(double radius)
{
  transform_radius=radius;
}

void PointCloudGenerator::generatePose(Eigen::Matrix3d& R, Eigen::Vector3d& t)
{
  double phi=randomNumber(0,2*M_PI);
  double theta=randomNumber(0,M_PI);
  double psi=randomNumber(0,2*M_PI);
  
  R(0,0)=cos(psi)*cos(phi)-cos(theta)*sin(phi)*sin(psi);
  R(0,1)=cos(psi)*sin(phi)+cos(theta)*cos(phi)*sin(psi);
  R(0,2)=sin(psi)*sin(theta);
  
  R(1,0)=-sin(psi)*cos(phi)-cos(theta)*sin(phi)*cos(psi);
  R(1,1)=-sin(psi)*sin(phi)+cos(theta)*cos(phi)*cos(psi);
  R(1,2)=cos(psi)*sin(theta);
  
  R(2,0)=sin(theta)*sin(phi);
  R(2,1)=-sin(theta)*cos(phi);
  R(2,2)=cos(theta);
  
  t[0]=randomNumber(-transform_radius,transform_radius);
  t[1]=randomNumber(-transform_radius,transform_radius);
  t[2]=randomNumber(-transform_radius,transform_radius);
}
void PointCloudGenerator::generatePoint(Eigen::Vector3d& point)
{
  double theta=randomNumber(0,M_PI);
  double phi=randomNumber(0,2*M_PI);
  double R=randomNumber(0,point_radius);
  
  point[0]=sin(theta)*sin(phi)*R;
  point[1]=-sin(theta)*cos(phi)*R;
  point[2]=cos(theta)*R;
}
void PointCloudGenerator::projectWithNoise(const Eigen::Matrix3d& R, const Eigen::Vector3d& t, const Eigen::Vector3d& point3d, Eigen::Vector2d& point2d)
{
  Eigen::Vector3d point3d2;
  
  point3d2=R*point3d+t;
  
  double nu=randomNumber(-project_noise,project_noise);
  double nv=randomNumber(-project_noise,project_noise);
  
  point2d[0]=cx+fx*point3d2[0]/point3d2[2]+nu;
  point2d[1]=cy+fy*point3d2[1]/point3d2[2]+nv; 
}
void PointCloudGenerator::projectWithoutNoise(const Eigen::Matrix3d& R, const Eigen::Vector3d& t, const Eigen::Vector3d& point3d, Eigen::Vector2d& point2d)
{
  Eigen::Vector3d point3d2;
  
  point3d2=R*point3d+t;
   
  point2d[0]=cx+fx*point3d2[0]/point3d2[2];
  point2d[1]=cy+fy*point3d2[1]/point3d2[2];
}
void PointCloudGenerator::transformWithNoise(const Eigen::Matrix3d& R, const Eigen::Vector3d& t, const Eigen::Vector3d& point3d1, Eigen::Vector3d& point3d2)
{
   point3d2=R*point3d1+t;
}
void PointCloudGenerator::transformWithoutNoise(const Eigen::Matrix3d& R, const Eigen::Vector3d& t, const Eigen::Vector3d& point3d1, Eigen::Vector3d& point3d2)
{
  point3d2=R*point3d1+t;
  Eigen::Vector3d noise_vector;
  noise_vector[0]=randomNumber(-transform_noise,transform_noise);
  noise_vector[1]=randomNumber(-transform_noise,transform_noise);
  noise_vector[2]=randomNumber(-transform_noise,transform_noise);
  point3d2+=noise_vector;
}

