/****************************************************************************
 *
 * $Id: file.cpp 3496 2011-11-22 15:14:32Z fnovotny $
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2012 by INRIA. All rights reserved.
 * 
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional 
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 * 
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 * 
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Contact visp@irisa.fr if any conditions of this licensing are
 * not clear to you.
 *
 * Description:
 * 
 *
 * Authors:
 * Filip Novotny
 * 
 *
 *****************************************************************************/

/*!
 \file camera.h
 \brief 
 */
#include "ros/ros.h"
#include <sensor_msgs/Joy.h>
#include <actionlib_msgs/GoalID.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <geometry_msgs/Twist.h>
#include <string>

#ifndef __joystick_TELEOP_H__
#define __joystick_TELEOP_H__
namespace turtlesim_pioneer
{
class Teleop
{
private:
  ros::NodeHandle n_;
  ros::AsyncSpinner spinner;

  int linear_, angular_, cancel_;
  double l_scale_, a_scale_;

  actionlib_msgs::GoalID goal_;
  bool goal_set_;

  ros::Subscriber joy_subscriber_;
  ros::Subscriber goal_subscriber_;
  ros::Subscriber velocity_subscriber_;
  ros::Publisher velocity_publisher_;
  ros::Publisher goal_cancel_publisher_;
  unsigned int queue_size_;

  void velocityCallback(const geometry_msgs::Twist::ConstPtr& velocity);
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void goalCallback(const move_base_msgs::MoveBaseActionGoal::ConstPtr& goal);
public:
  Teleop(int argc, char**argv);
  virtual ~Teleop();
};
}
#endif /* CAMERA_H_ */
