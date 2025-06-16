#include "husky_controll/differential_calculations.h"

DifferentialCalculations::DifferentialCalculations(double wheel_radius, double dist_between_wheels)
{
  xx_ = 0.0;
  yy_ = 0.0;
  th_ = 0.0;
  wheel_radius_ = wheel_radius;
  dist_between_wheels_ = dist_between_wheels;
}

void DifferentialCalculations::getWheelsSpeed(const double& linear_vel, const double& angular_vel, double& vel_right_wheel, double& vel_left_wheel)
{
  vel_right_wheel = (-linear_vel - dist_between_wheels_ * angular_vel / 2.0) / wheel_radius_;
  vel_left_wheel = (linear_vel - dist_between_wheels_ * angular_vel / 2.0) / wheel_radius_;
}

void DifferentialCalculations::odometryStep(const double& dt_angle_right_wheel, const double& dt_angle_left_wheel)
{
  double dt_dist_right_wheel = dt_angle_right_wheel * wheel_radius_;
  double dt_dist_left_wheel = dt_angle_left_wheel * wheel_radius_;

  double dt_dist = +(dt_dist_left_wheel - dt_dist_right_wheel) / 2.0;
  double dt_ang  = -(dt_dist_left_wheel + dt_dist_right_wheel) / dist_between_wheels_;

  xx_ = xx_ + dt_dist * std::cos(th_ + dt_ang / 2.0);
  yy_ = yy_ + dt_dist * std::sin(th_ + dt_ang / 2.0);
  th_ = normalizeAngle(th_ + dt_ang);
}

void DifferentialCalculations::getRobotSpeed(const double& vel_right_wheel, const double& vel_left_wheel, double& linear_vel, double& angular_vel)
{
  double linear_vel_right_wheel = vel_right_wheel * wheel_radius_;
  double linear_vel_left_wheel = vel_left_wheel * wheel_radius_;
  linear_vel = (linear_vel_left_wheel - linear_vel_right_wheel) / 2.0;
  angular_vel = -(linear_vel_left_wheel + linear_vel_right_wheel) / dist_between_wheels_;
}