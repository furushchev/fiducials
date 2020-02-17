// -*- mode: C++ -*-
/*
 *  Copyright (c) 2020, GITAI Inc.
 *  All rights reserved.
 * aruco_detect_nodelet.hpp
 * Author: Yuki Furuta <me@furushchev.ru>
 */


#ifndef ARUCO_DETECT_NODELET_HPP__
#define ARUCO_DETECT_NODELET_HPP__

#include <nodelet/nodelet.h>
#include "aruco_detect/aruco_detect.hpp"

namespace aruco_detect
{
class ArucoDetectNodelet : public nodelet::Nodelet
{
  virtual ~ArucoDetectNodelet();

  virtual void onInit() override;

  aruco_detect::FiducialsNode *fiducials_node;
};
}

#endif // ARUCO_DETECT_NODELET_HPP__
