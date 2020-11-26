/*
 * Copyright (c) 2017-19, Ubiquity Robotics Inc., Austin Hendrix
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of the FreeBSD Project.
 *
 */

#include <assert.h>
#include <sys/time.h>
#include <unistd.h>

#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <list>
#include <string>
#include <boost/algorithm/string.hpp>

#include "aruco_detect/aruco_detect.hpp"

using namespace std;
using namespace cv;

namespace aruco_detect
{

/**
  * @brief Return object points for the system centered in a single marker, given the marker length
  */
static void getSingleMarkerObjectPoints(float markerLength, vector<Point3f>& objPoints) {

    CV_Assert(markerLength > 0);

    // set coordinate system in the middle of the marker, with Z pointing out
    objPoints.clear();
    objPoints.push_back(Vec3f(-markerLength / 2.f, markerLength / 2.f, 0));
    objPoints.push_back(Vec3f( markerLength / 2.f, markerLength / 2.f, 0));
    objPoints.push_back(Vec3f( markerLength / 2.f,-markerLength / 2.f, 0));
    objPoints.push_back(Vec3f(-markerLength / 2.f,-markerLength / 2.f, 0));
}

// Euclidean distance between two points
static double dist(const cv::Point2f &p1, const cv::Point2f &p2)
{
    double x1 = p1.x;
    double y1 = p1.y;
    double x2 = p2.x;
    double y2 = p2.y;

    double dx = x1 - x2;
    double dy = y1 - y2;

    return sqrt(dx*dx + dy*dy);
}

// Compute area in image of a fiducial, using Heron's formula
// to find the area of two triangles
static double calcFiducialArea(const std::vector<cv::Point2f> &pts)
{
    const Point2f &p0 = pts.at(0);
    const Point2f &p1 = pts.at(1);
    const Point2f &p2 = pts.at(2);
    const Point2f &p3 = pts.at(3);

    double a1 = dist(p0, p1);
    double b1 = dist(p0, p3);
    double c1 = dist(p1, p3);

    double a2 = dist(p1, p2);
    double b2 = dist(p2, p3);
    double c2 = c1;

    double s1 = (a1 + b1 + c1) / 2.0;
    double s2 = (a2 + b2 + c2) / 2.0;

    a1 = sqrt(s1*(s1-a1)*(s1-b1)*(s1-c1));
    a2 = sqrt(s2*(s2-a2)*(s2-b2)*(s2-c2));
    return a1+a2;
}

// estimate reprojection error
static double getReprojectionError(const vector<Point3f> &objectPoints,
                            const vector<Point2f> &imagePoints,
                            const Mat &cameraMatrix, const Mat  &distCoeffs,
                            const Vec3d &rvec, const Vec3d &tvec) {

    vector<Point2f> projectedPoints;

    cv::projectPoints(objectPoints, rvec, tvec, cameraMatrix,
                      distCoeffs, projectedPoints);

    // calculate RMS image error
    double totalError = 0.0;
    for (unsigned int i=0; i<objectPoints.size(); i++) {
        double error = dist(imagePoints[i], projectedPoints[i]);
        totalError += error*error;
    }
    double rerror = totalError/(double)objectPoints.size();
    return rerror;
}

void FiducialsNode::estimatePoseSingleMarkers(const vector<int> &ids,
                                const vector<vector<Point2f > >&corners,
                                float markerLength,
                                const cv::Mat &cameraMatrix,
                                const cv::Mat &distCoeffs,
                                vector<Vec3d>& rvecs, vector<Vec3d>& tvecs,
                                vector<double>& reprojectionError) {

    CV_Assert(markerLength > 0);

    vector<Point3f> markerObjPoints;
    int nMarkers = (int)corners.size();
    rvecs.reserve(nMarkers);
    tvecs.reserve(nMarkers);
    reprojectionError.reserve(nMarkers);

    // for each marker, calculate its pose
    for (int i = 0; i < nMarkers; i++) {
       double fiducialSize = markerLength;

       std::map<int, double>::iterator it = fiducialLens.find(ids[i]);
       if (it != fiducialLens.end()) {
          fiducialSize = it->second;
       }

       getSingleMarkerObjectPoints(fiducialSize, markerObjPoints);
       cv::solvePnP(markerObjPoints, corners[i], cameraMatrix, distCoeffs,
                    rvecs[i], tvecs[i]);

       reprojectionError[i] =
          getReprojectionError(markerObjPoints, corners[i],
                               cameraMatrix, distCoeffs,
                               rvecs[i], tvecs[i]);
    }
}

void FiducialsNode::configCallback(aruco_detect::DetectorParamsConfig & config, uint32_t level)
{
    boost::mutex::scoped_lock lock(mutex);
    detectorParams->adaptiveThreshConstant = config.adaptiveThreshConstant;
    detectorParams->adaptiveThreshWinSizeMin = config.adaptiveThreshWinSizeMin;
    detectorParams->adaptiveThreshWinSizeMax = config.adaptiveThreshWinSizeMax;
    detectorParams->adaptiveThreshWinSizeStep = config.adaptiveThreshWinSizeStep;
    detectorParams->cornerRefinementMaxIterations = config.cornerRefinementMaxIterations;
    detectorParams->cornerRefinementMinAccuracy = config.cornerRefinementMinAccuracy;
    detectorParams->cornerRefinementWinSize = config.cornerRefinementWinSize;
#if defined(GITAI_CV_ARUCO_ENABLED) || CV_MAJOR_VERSION<3 || (CV_MAJOR_VERSION<=3 && CV_MINOR_VERSION<=2)
    detectorParams->doCornerRefinement = config.doCornerRefinement;
#else
    if (config.doCornerRefinement) {
       if (config.cornerRefinementSubpix) {
         detectorParams->cornerRefinementMethod = aruco::CORNER_REFINE_SUBPIX;
       }
       else {
         detectorParams->cornerRefinementMethod = aruco::CORNER_REFINE_CONTOUR;
       }
    }
    else {
       detectorParams->cornerRefinementMethod = aruco::CORNER_REFINE_NONE;
    }
#endif
    detectorParams->errorCorrectionRate = config.errorCorrectionRate;
    detectorParams->minCornerDistanceRate = config.minCornerDistanceRate;
    detectorParams->markerBorderBits = config.markerBorderBits;
    detectorParams->maxErroneousBitsInBorderRate = config.maxErroneousBitsInBorderRate;
    detectorParams->minDistanceToBorder = config.minDistanceToBorder;
    detectorParams->minMarkerDistanceRate = config.minMarkerDistanceRate;
    detectorParams->minMarkerPerimeterRate = config.minMarkerPerimeterRate;
    detectorParams->maxMarkerPerimeterRate = config.maxMarkerPerimeterRate;
    detectorParams->minOtsuStdDev = config.minOtsuStdDev;
    detectorParams->perspectiveRemoveIgnoredMarginPerCell = config.perspectiveRemoveIgnoredMarginPerCell;
    detectorParams->perspectiveRemovePixelPerCell = config.perspectiveRemovePixelPerCell;
    detectorParams->polygonalApproxAccuracyRate = config.polygonalApproxAccuracyRate;
}

void FiducialsNode::ignoreCallback(const std_msgs::String& msg)
{
    ignoreIds.clear();
    pnh.setParam("ignore_fiducials", msg.data);
    handleIgnoreString(msg.data);
}

void FiducialsNode::camInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
    if (haveCamInfo) {
        return;
    }

    if (msg->K != boost::array<double, 9>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0})) {
        for (int i=0; i<3; i++) {
            for (int j=0; j<3; j++) {
                cameraMatrix.at<double>(i, j) = msg->K[i*3+j];
            }
        }

        for (int i=0; i<5; i++) {
            distortionCoeffs.at<double>(0,i) = msg->D[i];
        }

        haveCamInfo = true;
        frameId = msg->header.frame_id;
    }
    else {
        ROS_WARN("%s", "CameraInfo message has invalid intrinsics, K matrix all zeros");
    }
}

void FiducialsNode::imageCallback(const sensor_msgs::ImageConstPtr & msg) {
    boost::mutex::scoped_lock lock(mutex);
    ROS_INFO("Got image %d", msg->header.seq);
    frameNum++;

    cv_bridge::CvImagePtr cv_ptr;

    fiducial_msgs::FiducialTransformArray fta;
    fta.header.stamp = msg->header.stamp;
    fta.header.frame_id = frameId;
    fta.image_seq = msg->header.seq;

    fiducial_msgs::FiducialArray fva;
    fva.header.stamp = msg->header.stamp;
    fva.header.frame_id =frameId;
    fva.image_seq = msg->header.seq;

    if (enable_debug) publishDebugImages(msg);

    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        vector <int>  ids;
        vector <vector <Point2f> > corners, rejected;
        vector <Vec3d>  rvecs, tvecs;

        aruco::detectMarkers(cv_ptr->image, dictionary, corners, ids, detectorParams);
        ROS_INFO("Detected %d markers", (int)ids.size());

        for (size_t i=0; i<ids.size(); i++) {
	    if (std::count(ignoreIds.begin(), ignoreIds.end(), ids[i]) != 0) {
	        ROS_INFO("Ignoring id %d", ids[i]);
	        continue;
	    }
            fiducial_msgs::Fiducial fid;
            fid.fiducial_id = ids[i];

            fid.x0 = corners[i][0].x;
            fid.y0 = corners[i][0].y;
            fid.x1 = corners[i][1].x;
            fid.y1 = corners[i][1].y;
            fid.x2 = corners[i][2].x;
            fid.y2 = corners[i][2].y;
            fid.x3 = corners[i][3].x;
            fid.y3 = corners[i][3].y;
            fva.fiducials.push_back(fid);
        }

        vertices_pub->publish(fva);

        if(ids.size() > 0) {
            aruco::drawDetectedMarkers(cv_ptr->image, corners, ids);
        }

        if (doPoseEstimation) {
            if (!haveCamInfo) {
                if (frameNum > 5) {
                    ROS_ERROR("No camera intrinsics");
                }
                return;
            }

            vector <double>reprojectionError;
            estimatePoseSingleMarkers(ids, corners, (float)fiducial_len,
                                      cameraMatrix, distortionCoeffs,
                                      rvecs, tvecs,
                                      reprojectionError);

            for (size_t i=0; i<ids.size(); i++) {
                aruco::drawAxis(cv_ptr->image, cameraMatrix, distortionCoeffs,
                                rvecs[i], tvecs[i], (float)fiducial_len);

                ROS_INFO("Detected id %d T %.2f %.2f %.2f R %.2f %.2f %.2f", ids[i],
                         tvecs[i][0], tvecs[i][1], tvecs[i][2],
                         rvecs[i][0], rvecs[i][1], rvecs[i][2]);

                if (std::count(ignoreIds.begin(), ignoreIds.end(), ids[i]) != 0) {
                    ROS_INFO("Ignoring id %d", ids[i]);
                    continue;
                }

                double angle = norm(rvecs[i]);
                Vec3d axis = rvecs[i] / angle;
                ROS_INFO("angle %f axis %f %f %f",
                         angle, axis[0], axis[1], axis[2]);

                fiducial_msgs::FiducialTransform ft;
                ft.fiducial_id = ids[i];

                ft.transform.translation.x = tvecs[i][0];
                ft.transform.translation.y = tvecs[i][1];
                ft.transform.translation.z = tvecs[i][2];

                tf2::Quaternion q;
                q.setRotation(tf2::Vector3(axis[0], axis[1], axis[2]), angle);

                ft.transform.rotation.w = q.w();
                ft.transform.rotation.x = q.x();
                ft.transform.rotation.y = q.y();
                ft.transform.rotation.z = q.z();

                ft.fiducial_area = calcFiducialArea(corners[i]);
                ft.image_error = reprojectionError[i];

                // Convert image_error (in pixels) to object_error (in meters)
                ft.object_error =
                    (reprojectionError[i] / dist(corners[i][0], corners[i][2])) *
                    (norm(tvecs[i]) / fiducial_len);

                fta.transforms.push_back(ft);
            }
            pose_pub->publish(fta);
        }

        if (publish_images) {
	    image_pub.publish(cv_ptr->toImageMsg());
        }
    }
    catch(cv_bridge::Exception & e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    catch(cv::Exception & e) {
        ROS_ERROR("cv exception: %s", e.what());
    }
}

void FiducialsNode::publishDebugImages(const sensor_msgs::ImageConstPtr &msg)
{
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  Mat corners_mat;
  vector < vector < int > > ids;
  Mat thresholds_mat;
  aruco::detectMarkers(cv_ptr->image, dictionary, corners_mat, ids, thresholds_mat, detectorParams);

  // corners_mat -> corners
  vector < vector < vector < Point2f > > > corners;
  {
    const int dims[3] = {corners_mat.size[0], corners_mat.size[1], corners_mat.size[2]};
    corners.resize(dims[0]);
    for (int i = 0; i < dims[0]; ++i)
    {
      auto corners_mat_i = corners_mat.row(i).reshape(0, 2, &dims[1]);
      corners[i].resize(ids[i].size());  // Assume: dims[1] >= ids[i].size()
      for (int j = 0; j < ids[i].size(); ++j)
      {
        auto corners_mat_ij = corners_mat_i.row(j).reshape(0, 1, &dims[2]);
        corners[i][j].resize(corners_mat_ij.cols);
        corners_mat_ij.copyTo(corners[i][j]);
      }
    }
  }

  // thresholds_mat -> thresholds
  vector < Mat > thresholds;
  {
    const int dims[3] = { thresholds_mat.size[0], thresholds_mat.size[1], thresholds_mat.size[2]};
    thresholds.resize(thresholds_mat.size[0]);
    for (int i = 0; i < dims[0]; ++i)
    {
      auto thresholds_mat_i = thresholds_mat.row(i).reshape(0, 2, &dims[1]);
      thresholds_mat_i.copyTo(thresholds[i]);
    }
  }

  int all_id_counts = 0;
  for (int i = 0; i < ids.size(); ++i)
  {
    all_id_counts += ids[i].size();
  }
  ROS_INFO("Detected %d markers using %d images", all_id_counts, (int)thresholds.size());

  bool advertised = false;
  while (debug_image_pubs.size() < thresholds.size())
  {
    std::ostringstream oss;
    oss << "debug_image_" << debug_image_pubs.size();
    debug_image_pubs.push_back(it.advertise(oss.str(), 1));
    ROS_INFO("Advertised %s", oss.str().c_str());
    advertised = true;
  }
  if (advertised)
  {
    ros::Rate(1.0).sleep();
  }

  for (int i = 0; i < thresholds.size(); ++i)
  {
    ROS_DEBUG("i=%d corners[i].size=%d ids[i].size=%d", i, (int)corners[i].size(), (int)ids[i].size());
    cv::cvtColor(thresholds[i], thresholds[i], cv::COLOR_GRAY2BGR);
    aruco::drawDetectedMarkers(thresholds[i], corners[i], ids[i]);
    cv_bridge::CvImage cv_image;
    cv_image.image = thresholds[i];
    cv_image.header = msg->header;
    cv_image.encoding = sensor_msgs::image_encodings::BGR8;
    debug_image_pubs[i].publish(cv_image.toImageMsg());
  }
}

void FiducialsNode::handleIgnoreString(const std::string& str)
{
    /*
    ignogre fiducials can take comma separated list of individual
    fiducial ids or ranges, eg "1,4,8,9-12,30-40"
    */
    std::vector<std::string> strs;
    boost::split(strs, str, boost::is_any_of(","));
    for (const string& element : strs) {
        if (element == "") {
           continue;
        }
        std::vector<std::string> range;
        boost::split(range, element, boost::is_any_of("-"));
        if (range.size() == 2) {
           int start = std::stoi(range[0]);
           int end = std::stoi(range[1]);
           ROS_INFO("Ignoring fiducial id range %d to %d", start, end);
           for (int j=start; j<=end; j++) {
               ignoreIds.push_back(j);
           }
        }
        else if (range.size() == 1) {
           int fid = std::stoi(range[0]);
           ROS_INFO("Ignoring fiducial id %d", fid);
           ignoreIds.push_back(fid);
        }
        else {
           ROS_ERROR("Malformed ignore_fiducials: %s", element.c_str());
        }
    }
}

bool FiducialsNode::enableDetectionsCallback(std_srvs::SetBool::Request &req,
                                std_srvs::SetBool::Response &res)
{
    enable_detections = req.data;

    if (enable_detections)
    {
        if (vertices_pub->getNumSubscribers() > 0 ||
            pose_pub->getNumSubscribers() > 0 ||
            image_pub.getNumSubscribers() > 0)
        {
            subscribe();
        }
    }
    else
    {
        unsubscribe();
    }


    if (enable_detections){
        res.message = "Enabled aruco detections.";
        ROS_INFO("Enabled aruco detections.");
    }
    else {
        res.message = "Disabled aruco detections.";
        ROS_INFO("Disabled aruco detections.");
    }
    
    res.success = true;
    return true;
}

void FiducialsNode::subscriberConnectionCallback(const ros::SingleSubscriberPublisher &ssp)
{
    subscriberConnectionCallback();
}

void FiducialsNode::imageSubscriberConnectionCallback(
    const image_transport::SingleSubscriberPublisher &ssp)
{
    subscriberConnectionCallback();
}

void FiducialsNode::subscriberConnectionCallback()
{
    if (!enable_detections) return;
    if (vertices_pub->getNumSubscribers() > 0 ||
        pose_pub->getNumSubscribers() > 0 ||
        image_pub.getNumSubscribers() > 0)
    {
        subscribe();
    }
    else if (vertices_pub->getNumSubscribers() == 0 &&
             pose_pub->getNumSubscribers() == 0 &&
             image_pub.getNumSubscribers() == 0)
    {
        unsubscribe();
    }
}

void FiducialsNode::subscribe()
{
    if (!subscribe_topics)
    {
        subscribe_topics = true;

        img_sub = it.subscribe("camera", 1,
                               &FiducialsNode::imageCallback, this);
        caminfo_sub = pnh.subscribe("camera_info", 1,
                                   &FiducialsNode::camInfoCallback, this);
        ROS_INFO("subscribed camera/camera_info topics");
    }
}

void FiducialsNode::unsubscribe()
{
    if (subscribe_topics)
    {
        subscribe_topics = false;

        img_sub.shutdown();
        caminfo_sub.shutdown();
        ROS_INFO("unsubscribed camera/camera_info topics");
    }
}

FiducialsNode::FiducialsNode(ros::NodeHandle nh_, ros::NodeHandle pnh_) : nh(nh_), pnh(pnh_), it(pnh_), configServer(pnh)
{
    frameNum = 0;

    // Camera intrinsics
    cameraMatrix = cv::Mat::zeros(3, 3, CV_64F);

    // distortion coefficients
    distortionCoeffs = cv::Mat::zeros(1, 5, CV_64F);

    haveCamInfo = false;
    enable_detections = true;
    subscribe_topics = false;

    int dicno;

    detectorParams = new aruco::DetectorParameters();
    callbackType = boost::bind(&FiducialsNode::configCallback, this, _1, _2);
    configServer.setCallback(callbackType);

    pnh.param<bool>("publish_images", publish_images, false);
    pnh.param<double>("fiducial_len", fiducial_len, 0.14);
    pnh.param<int>("dictionary", dicno, 7);
    pnh.param<bool>("do_pose_estimation", doPoseEstimation, true);

    std::string str;
    std::vector<std::string> strs;

    pnh.param<string>("ignore_fiducials", str, "");
    handleIgnoreString(str);

    /*
    fiducial size can take comma separated list of size: id or size: range,
    e.g. "200.0: 12, 300.0: 200-300"
    */
    pnh.param<string>("fiducial_len_override", str, "");
    boost::split(strs, str, boost::is_any_of(","));
    for (const string& element : strs) {
        if (element == "") {
           continue;
        }
        std::vector<std::string> parts;
        boost::split(parts, element, boost::is_any_of(":"));
        if (parts.size() == 2) {
            double len = std::stod(parts[1]);
            std::vector<std::string> range;
            boost::split(range, element, boost::is_any_of("-"));
            if (range.size() == 2) {
               int start = std::stoi(range[0]);
               int end = std::stoi(range[1]);
               ROS_INFO("Setting fiducial id range %d - %d length to %f",
                        start, end, len);
               for (int j=start; j<=end; j++) {
                   fiducialLens[j] = len;
               }
            }
            else if (range.size() == 1){
               int fid = std::stoi(range[0]);
               ROS_INFO("Setting fiducial id %d length to %f", fid, len);
               fiducialLens[fid] = len;
            }
            else {
               ROS_ERROR("Malformed fiducial_len_override: %s", element.c_str());
            }
        }
        else {
           ROS_ERROR("Malformed fiducial_len_override: %s", element.c_str());
        }
    }

    pnh.param<bool>("enable_debug", enable_debug, false);

    image_transport::SubscriberStatusCallback it_conn_cb = boost::bind(
        &FiducialsNode::imageSubscriberConnectionCallback, this, _1);
    ros::SubscriberStatusCallback conn_cb = boost::bind(&FiducialsNode::subscriberConnectionCallback,
                                                        this, _1);
    image_pub = it.advertise("fiducial_images", 1, it_conn_cb, it_conn_cb);


    vertices_pub = new ros::Publisher(pnh.advertise<fiducial_msgs::FiducialArray>(
                                          "fiducial_vertices", 1, conn_cb, conn_cb));

    pose_pub = new ros::Publisher(pnh.advertise<fiducial_msgs::FiducialTransformArray>(
                                      "fiducial_transforms", 1, conn_cb, conn_cb));

    dictionary = aruco::getPredefinedDictionary(dicno);

    ignore_sub = pnh.subscribe("ignore_fiducials", 1,
                              &FiducialsNode::ignoreCallback, this);

    service_enable_detections = pnh.advertiseService("enable_detections",
                        &FiducialsNode::enableDetectionsCallback, this);

    ROS_INFO("Aruco detection ready");
}
}
