#include "path_manager_example.h"
#include "ros/ros.h"
#include <cmath>

namespace rosplane
{

path_manager_example::path_manager_example() : path_manager_base()
{
  fil_state_ = fillet_state::STRAIGHT;
  dub_state_ = dubin_state::FIRST;
}

void path_manager_example::manage(const params_s &params, const input_s &input, output_s &output)
{

  if (num_waypoints_ < 2)
  {
    ROS_WARN_THROTTLE(4, "No waypoints received! Loitering about origin at 50m");
    output.is_straight = false;
    output.airspeed_command = 12;
    output.center[0] = 0.0f;
    output.center[1] = 0.0f;
    output.center[2] = -50.0f;
    output.orbit_radius = params.R_min;
    output.orbit_direction = 1;
  }
  else
  {
    if (waypoints_[last_waypoint_index_].course_valid)
    {
      manage_dubins(params, input, output);
    }
    else
    {
      /** Switch the following for flying directly to waypoints, or filleting corners */
      //manage_line(params, input, output);
      manage_fillet(params, input, output);
    }
  }
}

void path_manager_example::manage_line(const params_s &params, const input_s &input, output_s &output)
{

  Eigen::Vector3f position;
  position << input.position_north, input.position_east, -input.altitude;

  int next_waypoint_index;
  int idx_c;
  if (last_waypoint_index_ == num_waypoints_ - 1)
  {
    next_waypoint_index = 0;
    idx_c = 1;
  }
  else if (last_waypoint_index_ == num_waypoints_ - 2)
  {
    next_waypoint_index = num_waypoints_ - 1;
    idx_c = 0;
  }
  else
  {
    next_waypoint_index = last_waypoint_index_ + 1;
    idx_c = next_waypoint_index + 1;
  }

  Eigen::Vector3f last_waypoint(waypoints_[last_waypoint_index_].position);
  Eigen::Vector3f w_i(waypoints_[next_waypoint_index].position);
  Eigen::Vector3f next_waypoint(waypoints_[idx_c].position);

  output.is_straight = true;
  output.airspeed_command = waypoints_[last_waypoint_index_].airspeed_command;
  output.line_origin[0] = last_waypoint(0);
  output.line_origin[1] = last_waypoint(1);
  output.line_origin[2] = last_waypoint(2);
  Eigen::Vector3f leg1_unit_direction = (w_i - last_waypoint).normalized();
  Eigen::Vector3f q_i = (next_waypoint - w_i).normalized();
  output.line_direction[0] = leg1_unit_direction(0);
  output.line_direction[1] = leg1_unit_direction(1);
  output.line_direction[2] = leg1_unit_direction(2);

  Eigen::Vector3f n_i = (leg1_unit_direction + q_i).normalized();
  if ((position - w_i).dot(n_i) > 0.0f)
  {
    if (last_waypoint_index_ == num_waypoints_ - 1)
        last_waypoint_index_ = 0;
    else
      last_waypoint_index_++;
  }

}

void path_manager_example::manage_fillet(const params_s &params, const input_s &input, output_s &output)
{
  if (num_waypoints_ < 3) //since it fillets don't make sense between just two points
  {
    manage_line(params, input, output);
    return;
  }

  Eigen::Vector3f p;
  p << input.position_north, input.position_east, -input.altitude;

  int current_waypoint_index;
  int next_waypoint_index;
  if (last_waypoint_index_ == num_waypoints_ - 1)
  {
    current_waypoint_index = 0;
    next_waypoint_index = 1;
  }
  else if (last_waypoint_index_ == num_waypoints_ - 2)
  {
    current_waypoint_index = num_waypoints_ - 1;
    next_waypoint_index = 0;
  }
  else
  {
    current_waypoint_index = last_waypoint_index_ + 1;
    next_waypoint_index = current_waypoint_index + 1;
  }

  Eigen::Vector3f last_waypoint(waypoints_[last_waypoint_index_].position);
  Eigen::Vector3f current_waypoint(waypoints_[current_waypoint_index].position);
  Eigen::Vector3f next_waypoint(waypoints_[next_waypoint_index].position);

  float R_min = params.R_min;

  output.airspeed_command = waypoints_[last_waypoint_index_].airspeed_command;
  output.line_origin[0] = last_waypoint(0);
  output.line_origin[1] = last_waypoint(1);
  output.line_origin[2] = last_waypoint(2);
  Eigen::Vector3f leg1_unit_direction = (current_waypoint - last_waypoint).normalized();
  Eigen::Vector3f leg2_unit_direction = (next_waypoint - current_waypoint).normalized();
  float turn_angle_rad = acosf(-leg1_unit_direction.dot(leg2_unit_direction));

  Eigen::Vector3f z;
  switch (fil_state_)
  {
  case fillet_state::STRAIGHT:
    output.is_straight = true;
    output.line_direction[0] = leg1_unit_direction(0);
    output.line_direction[1] = leg1_unit_direction(1);
    output.line_direction[2] = leg1_unit_direction(2);
    output.center[0] = 1;
    output.center[1] = 1;
    output.center[2] = 1;
    output.orbit_radius = 1;
    output.orbit_direction = 1;
    z = current_waypoint - leg1_unit_direction*(R_min/tanf(turn_angle_rad/2.0));
    output.line_end[0] = current_waypoint(0);
    output.line_end[1] = current_waypoint(1);
    output.line_end[2] = current_waypoint(2);
    if ((p - z).dot(leg1_unit_direction) > 0)
      fil_state_ = fillet_state::ORBIT;
    break;
  case fillet_state::ORBIT:
    output.is_straight = false;
    output.line_direction[0] = leg2_unit_direction(0);
    output.line_direction[1] = leg2_unit_direction(1);
    output.line_direction[2] = leg2_unit_direction(2);
    Eigen::Vector3f c = current_waypoint - (leg1_unit_direction - leg2_unit_direction).normalized()*(R_min/sinf(turn_angle_rad/2.0));
    output.center[0] = c(0);
    output.center[1] = c(1);
    output.center[2] = c(2);
    output.orbit_radius = R_min;
    output.orbit_direction = ((leg1_unit_direction(0) * leg2_unit_direction(1) - leg1_unit_direction(1) * leg2_unit_direction(0)) > 0 ? 1 : -1);
    z = current_waypoint + leg2_unit_direction*(R_min/tanf(turn_angle_rad/2.0));
    if ((p - z).dot(leg2_unit_direction) > 0)
    {
      if (last_waypoint_index_ == num_waypoints_ - 1)
          last_waypoint_index_ = 0;
      else
        last_waypoint_index_++;
      fil_state_ = fillet_state::STRAIGHT;
    }
    break;
  }
}

void path_manager_example::manage_dubins(const params_s &params, const input_s &input, output_s &output)
{
  Eigen::Vector3f p;
  p << input.position_north, input.position_east, -input.altitude;

  output.airspeed_command = waypoints_[last_waypoint_index_].airspeed_command;
  output.line_origin[0] = 0;
  output.line_origin[1] = 0;
  output.line_origin[2] = 0;
  output.line_direction[0] = 0;
  output.line_direction[1] = 0;
  output.line_direction[2] = 0;
  output.center[0] = 0;
  output.center[1] = 0;
  output.center[2] = 0;

  switch (dub_state_)
  {
  case dubin_state::FIRST:
    dubinsParameters(waypoints_[0], waypoints_[1], params.R_min);
    output.is_straight = false;
    output.center[0] = dubinspath_.cs(0);
    output.center[1] = dubinspath_.cs(1);
    output.center[2] = dubinspath_.cs(2);
    output.orbit_radius = dubinspath_.R;
    output.orbit_direction = dubinspath_.lams;
    if ((p - dubinspath_.w1).dot(dubinspath_.q1) >= 0) // start in H1
    {
      dub_state_ = dubin_state::BEFORE_H1_WRONG_SIDE;
    }
    else
    {
      dub_state_ = dubin_state::BEFORE_H1;
    }
    break;
  case dubin_state::BEFORE_H1:
    output.is_straight = false;
    output.center[0] = dubinspath_.cs(0);
    output.center[1] = dubinspath_.cs(1);
    output.center[2] = dubinspath_.cs(2);
    output.orbit_radius = dubinspath_.R;
    output.orbit_direction = dubinspath_.lams;
    if ((p - dubinspath_.w1).dot(dubinspath_.q1) >= 0) // entering H1
    {
      dub_state_ = dubin_state::STRAIGHT;
    }
    break;
  case dubin_state::BEFORE_H1_WRONG_SIDE:
    output.is_straight = false;
    output.center[0] = dubinspath_.cs(0);
    output.center[1] = dubinspath_.cs(1);
    output.center[2] = dubinspath_.cs(2);
    output.orbit_radius = dubinspath_.R;
    output.orbit_direction = dubinspath_.lams;
    if ((p - dubinspath_.w1).dot(dubinspath_.q1) < 0) // exit H1
    {
      dub_state_ = dubin_state::BEFORE_H1;
    }
    break;
  case dubin_state::STRAIGHT:
    output.is_straight = true;
    output.line_origin[0] = dubinspath_.w1(0);
    output.line_origin[1] = dubinspath_.w1(1);
    output.line_origin[2] = dubinspath_.w1(2);
    // output.r[0] = dubinspath_.z1(0);
    // output.r[1] = dubinspath_.z1(1);
    // output.r[2] = dubinspath_.z1(2);
    output.line_direction[0] = dubinspath_.q1(0);
    output.line_direction[1] = dubinspath_.q1(1);
    output.line_direction[2] = dubinspath_.q1(2);
    output.orbit_radius = 1;
    output.orbit_direction = 1;
    if ((p - dubinspath_.w2).dot(dubinspath_.q1) >= 0) // entering H2
    {
      if ((p - dubinspath_.w3).dot(dubinspath_.q3) >= 0) // start in H3
      {
        dub_state_ = dubin_state::BEFORE_H3_WRONG_SIDE;
      }
      else
      {
        dub_state_ = dubin_state::BEFORE_H3;
      }
    }
    break;
  case dubin_state::BEFORE_H3:
    output.is_straight = false;
    output.center[0] = dubinspath_.ce(0);
    output.center[1] = dubinspath_.ce(1);
    output.center[2] = dubinspath_.ce(2);
    output.orbit_radius = dubinspath_.R;
    output.orbit_direction = dubinspath_.lame;
    if ((p - dubinspath_.w3).dot(dubinspath_.q3) >= 0) // entering H3
    {
      // increase the waypoint pointer
      int next_waypoint_index;
      if (last_waypoint_index_ == num_waypoints_ - 1)
      {
          last_waypoint_index_ = 0;
        next_waypoint_index = 1;
      }
      else if (last_waypoint_index_ == num_waypoints_ - 2)
      {
        last_waypoint_index_++;
        next_waypoint_index = 0;
      }
      else
      {
        last_waypoint_index_++;
        next_waypoint_index = last_waypoint_index_ + 1;
      }

      // plan new Dubin's path to next waypoint configuration
      dubinsParameters(waypoints_[last_waypoint_index_], waypoints_[next_waypoint_index], params.R_min);

      //start new path
      if ((p - dubinspath_.w1).dot(dubinspath_.q1) >= 0) // start in H1
      {
        dub_state_ = dubin_state::BEFORE_H1_WRONG_SIDE;
      }
      else
      {
        dub_state_ = dubin_state::BEFORE_H1;
      }
    }
    break;
  case dubin_state::BEFORE_H3_WRONG_SIDE:
    output.is_straight = false;
    output.center[0] = dubinspath_.ce(0);
    output.center[1] = dubinspath_.ce(1);
    output.center[2] = dubinspath_.ce(2);
    output.orbit_radius = dubinspath_.R;
    output.orbit_direction = dubinspath_.lame;
    if ((p - dubinspath_.w3).dot(dubinspath_.q3) < 0) // exit H3
    {
      dub_state_ = dubin_state::BEFORE_H1;
    }
    break;
  }
}

Eigen::Matrix3f path_manager_example::rotz(float theta)
{
  Eigen::Matrix3f R;
  R << cosf(theta), -sinf(theta), 0,
  sinf(theta),  cosf(theta), 0,
  0,            0, 1;

  return R;
}

float path_manager_example::mo(float in)
{
  float val;
  if (in > 0)
    val = fmod(in, 2.0*M_PI_F);
  else
  {
    float n = floorf(in/2.0/M_PI_F);
    val = in - n*2.0*M_PI_F;
  }
  return val;
}

void path_manager_example::dubinsParameters(const waypoint_s start_node, const waypoint_s end_node, float R)
{
  float ell = sqrtf((start_node.position[0] - end_node.position[0]) * (start_node.position[0] - end_node.position[0]) +
                    (start_node.position[1] - end_node.position[1]) * (start_node.position[1] - end_node.position[1]));
  if (ell < 2.0*R)
  {
    ROS_ERROR("The distance between nodes must be larger than 2R.");
  }
  else
  {
    dubinspath_.ps(0) = start_node.position[0];
    dubinspath_.ps(1) = start_node.position[1];
    dubinspath_.ps(2) = start_node.position[2];
    dubinspath_.chis = start_node.course_command;
    dubinspath_.pe(0) = end_node.position[0];
    dubinspath_.pe(1) = end_node.position[1];
    dubinspath_.pe(2) = end_node.position[2];
    dubinspath_.chie = end_node.course_command;


    Eigen::Vector3f crs = dubinspath_.ps;
    crs(0) += R*(cosf(M_PI_2_F)*cosf(dubinspath_.chis) - sinf(M_PI_2_F)*sinf(dubinspath_.chis));
    crs(1) += R*(sinf(M_PI_2_F)*cosf(dubinspath_.chis) + cosf(M_PI_2_F)*sinf(dubinspath_.chis));
    Eigen::Vector3f cls = dubinspath_.ps;
    cls(0) += R*(cosf(-M_PI_2_F)*cosf(dubinspath_.chis) - sinf(-M_PI_2_F)*sinf(dubinspath_.chis));
    cls(1) += R*(sinf(-M_PI_2_F)*cosf(dubinspath_.chis) + cosf(-M_PI_2_F)*sinf(dubinspath_.chis));
    Eigen::Vector3f cre = dubinspath_.pe;
    cre(0) += R*(cosf(M_PI_2_F)*cosf(dubinspath_.chie) - sinf(M_PI_2_F)*sinf(dubinspath_.chie));
    cre(1) += R*(sinf(M_PI_2_F)*cosf(dubinspath_.chie) + cosf(M_PI_2_F)*sinf(dubinspath_.chie));
    Eigen::Vector3f cle = dubinspath_.pe;
    cle(0) += R*(cosf(-M_PI_2_F)*cosf(dubinspath_.chie) - sinf(-M_PI_2_F)*sinf(dubinspath_.chie));
    cle(1) += R*(sinf(-M_PI_2_F)*cosf(dubinspath_.chie) + cosf(-M_PI_2_F)*sinf(dubinspath_.chie));

    float theta, theta2;
    // compute L1
    theta = atan2f(cre(1) - crs(1), cre(0) - crs(0));
    float L1 = (crs - cre).norm() + R*mo(2.0*M_PI_F + mo(theta - M_PI_2_F) - mo(dubinspath_.chis - M_PI_2_F))
               + R*mo(2.0*M_PI_F + mo(dubinspath_.chie - M_PI_2_F) - mo(theta - M_PI_2_F));

    // compute L2
    ell = (cle - crs).norm();
    theta = atan2f(cle(1) - crs(1), cle(0) - crs(0));
    float L2;
    if (2.0*R > ell)
      L2 = 9999.0f;
    else
    {
      theta2 = theta - M_PI_2_F + asinf(2.0*R/ell);
      L2 = sqrtf(ell*ell - 4.0*R*R) + R*mo(2.0*M_PI_F + mo(theta2) - mo(dubinspath_.chis - M_PI_2_F))
           + R*mo(2.0*M_PI_F + mo(theta2 + M_PI_F) - mo(dubinspath_.chie + M_PI_2_F));
    }

    // compute L3
    ell = (cre - cls).norm();
    theta = atan2f(cre(1) - cls(1), cre(0) - cls(0));
    float L3;
    if (2.0*R > ell)
      L3 = 9999.0f;
    else
    {
      theta2 = acosf(2.0*R/ell);
      L3 = sqrtf(ell*ell - 4*R*R) + R*mo(2.0*M_PI_F + mo(dubinspath_.chis + M_PI_2_F) - mo(theta + theta2))
           + R*mo(2.0*M_PI_F + mo(dubinspath_.chie - M_PI_2_F) - mo(theta + theta2 - M_PI_F));
    }

    // compute L4
    theta = atan2f(cle(1) - cls(1), cle(0) - cls(0));
    float L4 = (cls - cle).norm() + R*mo(2.0*M_PI_F + mo(dubinspath_.chis + M_PI_2_F) - mo(theta + M_PI_2_F))
               + R*mo(2.0*M_PI_F + mo(theta + M_PI_2_F) - mo(dubinspath_.chie + M_PI_2_F));

    // L is the minimum distance
    int idx = 1;
    dubinspath_.L = L1;
    if (L2 < dubinspath_.L)
    {
      dubinspath_.L = L2;
      idx = 2;
    }
    if (L3 < dubinspath_.L)
    {
      dubinspath_.L = L3;
      idx = 3;
    }
    if (L4 < dubinspath_.L)
    {
      dubinspath_.L = L4;
      idx = 4;
    }

    Eigen::Vector3f e1;
    //        e1.zero();
    e1(0) = 1;
    e1(1) = 0;
    e1(2) = 0;
    switch (idx)
    {
    case 1:
      dubinspath_.cs = crs;
      dubinspath_.lams = 1;
      dubinspath_.ce = cre;
      dubinspath_.lame = 1;
      dubinspath_.q1 = (cre - crs).normalized();
      dubinspath_.w1 = dubinspath_.cs + (rotz(-M_PI_2_F)*dubinspath_.q1)*R;
      dubinspath_.w2 = dubinspath_.ce + (rotz(-M_PI_2_F)*dubinspath_.q1)*R;
      break;
    case 2:
      dubinspath_.cs = crs;
      dubinspath_.lams = 1;
      dubinspath_.ce = cle;
      dubinspath_.lame = -1;
      ell = (cle - crs).norm();
      theta = atan2f(cle(1) - crs(1), cle(0) - crs(0));
      theta2 = theta - M_PI_2_F + asinf(2.0*R/ell);
      dubinspath_.q1 = rotz(theta2 + M_PI_2_F)*e1;
      dubinspath_.w1 = dubinspath_.cs + (rotz(theta2)*e1)*R;
      dubinspath_.w2 = dubinspath_.ce + (rotz(theta2 + M_PI_F)*e1)*R;
      break;
    case 3:
      dubinspath_.cs = cls;
      dubinspath_.lams = -1;
      dubinspath_.ce = cre;
      dubinspath_.lame = 1;
      ell = (cre - cls).norm();
      theta = atan2f(cre(1) - cls(1), cre(0) - cls(0));
      theta2 = acosf(2.0*R/ ell);
      dubinspath_.q1 = rotz(theta + theta2 - M_PI_2_F)*e1;
      dubinspath_.w1 = dubinspath_.cs + (rotz(theta + theta2)*e1)*R;
      dubinspath_.w2 = dubinspath_.ce + (rotz(theta + theta2 - M_PI_F)*e1)*R;
      break;
    case 4:
      dubinspath_.cs = cls;
      dubinspath_.lams = -1;
      dubinspath_.ce = cle;
      dubinspath_.lame = -1;
      dubinspath_.q1 = (cle - cls).normalized();
      dubinspath_.w1 = dubinspath_.cs + (rotz(M_PI_2_F)*dubinspath_.q1)*R;
      dubinspath_.w2 = dubinspath_.ce + (rotz(M_PI_2_F)*dubinspath_.q1)*R;
      break;
    }
    dubinspath_.w3 = dubinspath_.pe;
    dubinspath_.q3 = rotz(dubinspath_.chie)*e1;
    dubinspath_.R = R;
  }
}

}//end namespace
