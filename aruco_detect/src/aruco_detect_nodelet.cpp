// -*- mode: C++ -*-
/*
 *  Copyright (c) 2020, GITAI Inc.
 *  All rights reserved.
 * aruco_detect_nodelet.cpp
 * Author: Yuki Furuta <me@furushchev.ru>
 */

#include "aruco_detect/aruco_detect_nodelet.hpp"

namespace aruco_detect
{
void ArucoDetectNodelet::onInit()
{
  fiducials_node = new aruco_detect::FiducialsNode(
      getNodeHandle(), getPrivateNodeHandle());
}

ArucoDetectNodelet::~ArucoDetectNodelet()
{
  if (fiducials_node) delete fiducials_node;
}
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aruco_detect::ArucoDetectNodelet, nodelet::Nodelet)
