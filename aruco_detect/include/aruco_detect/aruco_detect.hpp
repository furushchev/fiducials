// -*- mode: C++ -*-
/*
 *  Copyright (c) 2020, GITAI Inc.
 *  All rights reserved.
 * aruco_detect.hpp
 * Author: Yuki Furuta <me@furushchev.ru>
 */


#ifndef ARUCO_DETECT_HPP__
#define ARUCO_DETECT_HPP__

#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/String.h>

#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>

#include "fiducial_msgs/Fiducial.h"
#include "fiducial_msgs/FiducialArray.h"
#include "fiducial_msgs/FiducialTransform.h"
#include "fiducial_msgs/FiducialTransformArray.h"
#include "aruco_detect/DetectorParamsConfig.h"

namespace aruco_detect
{

class FiducialsNode {
  private:
    ros::Publisher * vertices_pub;
    ros::Publisher * pose_pub;

    ros::Subscriber caminfo_sub;
    ros::Subscriber ignore_sub;
    image_transport::ImageTransport it;
    image_transport::Subscriber img_sub;

    ros::ServiceServer service_enable_detections;

    // if set, we publish the images that contain fiducials
    bool publish_images;
    bool enable_detections;
    bool subscribe_topics;

    double fiducial_len;

    bool doPoseEstimation;
    bool haveCamInfo;
    cv::Mat cameraMatrix;
    cv::Mat distortionCoeffs;
    int frameNum;
    std::string frameId;
    std::vector<int> ignoreIds;
    std::map<int, double> fiducialLens;
    ros::NodeHandle nh;
    ros::NodeHandle pnh;

    image_transport::Publisher image_pub;

    boost::mutex mutex;
    cv::Ptr<cv::aruco::DetectorParameters> detectorParams;
    cv::Ptr<cv::aruco::Dictionary> dictionary;

    void handleIgnoreString(const std::string& str);

    void estimatePoseSingleMarkers(const std::vector<int> &ids,
                                   const std::vector<std::vector<cv::Point2f > >&corners,
                                   float markerLength,
                                   const cv::Mat &cameraMatrix,
                                   const cv::Mat &distCoeffs,
                                   std::vector<cv::Vec3d>& rvecs, std::vector<cv::Vec3d>& tvecs,
                                   std::vector<double>& reprojectionError);


    void ignoreCallback(const std_msgs::String &msg);
    void imageCallback(const sensor_msgs::ImageConstPtr &msg);
    void camInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &msg);
    void configCallback(aruco_detect::DetectorParamsConfig &config, uint32_t level);

    bool enableDetectionsCallback(std_srvs::SetBool::Request &req,
                                  std_srvs::SetBool::Response &res);

    void subscribe();
    void unsubscribe();
    void subscriberConnectionCallback(const ros::SingleSubscriberPublisher &ssp);
    void imageSubscriberConnectionCallback(const image_transport::SingleSubscriberPublisher &ssp);
    void subscriberConnectionCallback();

    dynamic_reconfigure::Server<aruco_detect::DetectorParamsConfig> configServer;
    dynamic_reconfigure::Server<aruco_detect::DetectorParamsConfig>::CallbackType callbackType;

  public:
  FiducialsNode(ros::NodeHandle nh  = ros::NodeHandle(),
                ros::NodeHandle pnh = ros::NodeHandle("~"));
};

}

#endif // ARUCO_DETECT_HPP__
