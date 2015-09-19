//Include ROS libraries for node messages
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/forwards.h>
#include <ros/single_subscriber_publisher.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>

// Include OpenCV for images and viewer
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>

//Include Boost tools for detection arrays and threads
#include <boost/thread.hpp>
#include <boost/unordered_set.hpp>
#include <boost/unordered_map.hpp>
#include <boost/make_shared.hpp>

#include "ros_caffe/Classifier.h"

// Define the default topic names
const std::string DEFAULT_IMAGE_TOPIC       = "image";
const std::string DEFAULT_CAMERA_INFO_TOPIC = "camera_info";
const std::string DEFAULT_DETECTIONS_TOPIC  = "detections";

// ROS parts
ros::NodeHandlePtr node_;
boost::shared_ptr<image_transport::ImageTransport> image_;
sensor_msgs::CameraInfo camera_info_;

// ROS Publishers and Subscribers
ros::Publisher detection_publisher_;
ros::Subscriber info_subscriber;
image_transport::Subscriber image_subscriber;

// Config
//ros_caffe::ros_caffe config_;
//typedef ros_caffe::ros_caffe Config;

// EBT plugin perams
//int ebt_maxd_;

// EBT plugin global
//Eigen::Matrix4d pose_;

// Settings and local information
bool running_;
bool has_camera_info_;
bool quit_;

// Package Functions

// Callback for camera info
void InfoCallback(const sensor_msgs::CameraInfoConstPtr& camera_info);

// Callback for new subcription
void ConnectCallback(const ros::SingleSubscriberPublisher& info);

// Callback for unsubcription
void DisconnectCallback(const ros::SingleSubscriberPublisher& info);

// Handler for unsubcription
void DisconnectHandler();

// Read Parameters for unsubcription
void GetParameterValues();

// Advertize Publishers
void SetupPublisher();

// Initialize classifier using Parameters
void InitializeClassifier();

// Initialize the ROS Node
void InitializeROSNode();
