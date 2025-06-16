#pragma once

#include <cmath>
#include "math_utilities.h"

class DifferentialCalculations
{
public:
  DifferentialCalculations(double wheel_radius, double dist_between_wheels);

  void getWheelsSpeed(const double& linear_vel, const double& angular_vel, double& vel_right_wheel, double& vel_left_wheel);
  void getRobotSpeed(const double& vel_right_wheel, const double& vel_left_wheel, double& linear_vel, double& angular_vel);
  void odometryStep(const double& dt_angle_right_wheel, const double& dt_angle_left_wheel);

  double xx_;
  double yy_;
  double th_;
private:
  double wheel_radius_;
  double dist_between_wheels_;
};