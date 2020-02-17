// -*- mode: C++ -*-
/*
 *  Copyright (c) 2020, GITAI Inc.
 *  All rights reserved.
 * aruco_detect_node.cpp
 * Author: Yuki Furuta <me@furushchev.ru>
 */

#include "aruco_detect/aruco_detect.hpp"

int main(int argc, char ** argv) {
    ros::init(argc, argv, "aruco_detect");

    aruco_detect::FiducialsNode* node = new aruco_detect::FiducialsNode();

    ros::spin();

    return 0;
}
