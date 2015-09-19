//Include the base of ros_caffe
#include "ros_caffe/ros_caffe.h"

using namespace std;

void InfoCallback(const sensor_msgs::CameraInfoConstPtr& camera_info)
{
    camera_info_ = (*camera_info);
    has_camera_info_ = true;
}

void ImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    /*
    // Dont atempt to use the image without having info about the camera first
    //    if(!has_camera_info_){
    //        ROS_WARN("No Camera Info Received Yet");
    //        return;
    //    }

    // Get the image
    cv_bridge::CvImagePtr subscribed_ptr;
    try
    {
        subscribed_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat subscribed_gray = subscribed_ptr->image;


    // Apply the tracker to the image
    ApplyTrackerSettingsCallback(tracker_);
    tracker_->setPose(pose_cv);
    ProcessUserActions();
    tracker_->setImage(subscribed_gray);
    if(ebt_reset_){
        tracker_->init_ = false;
        ROS_INFO("ObjectTrackin2D message input: RESET\n");
        ebt_reset_ = false;
    }
    if(ebt_auto_init_){
        ebt_init_ = tracker_->init_;
    }
    if(ebt_init_){
        tracker_->init_ = true;
        tracker_->initialize();
        ROS_INFO("ObjectTrackin2D message input: REINIT\n");
        ebt_init_ = tracker_->init_;
    }

    if(ebt_init_)
        return;

    tracker_->tracking();


    pose_cv = tracker_->getPose();
    cov_cv = tracker_->getCovariance();


    // Store the detection into an array struture
    ObjectDetection d;
    d.pose = pose_;
    d.good = !tracker_->init_;
    d.id = 0;
    d.ns = ebt_obj_id_;
    ObjectDetectionArray detections;
    detections.push_back(d);

    // Store the detection into an array struture
    visualization_msgs::MarkerArray marker_transforms;
    object_tracking_2d_ros::ObjectDetections object_detections;
    object_detections.header.frame_id = msg->header.frame_id;
    object_detections.header.stamp = msg->header.stamp;

    // Loop over each detection
    for(unsigned int i = 0; i < detections.size(); ++i)
    {
//        // skip bad detections
//        if(!detections[i].good)
//        {
//            continue;
//        }

        // Get quaternion for the marker
        Eigen::Matrix4d pose = GetDetectionTransform(detections[i]);
        Eigen::Matrix3d R = pose.block<3,3>(0,0);
        Eigen::Quaternion<double> q(R);

        // Set the attributes for the marker
        visualization_msgs::Marker marker_transform;
        marker_transform.header.frame_id = msg->header.frame_id;
        marker_transform.header.stamp = msg->header.stamp;
//        stringstream convert;
//        convert << "tag" << detections[i].id;
        marker_transform.id = detections[i].id;
        marker_transform.ns = detections[i].ns;

        // Set the object mesh for the marker
        marker_transform.type = visualization_msgs::Marker::MESH_RESOURCE;
        marker_transform.mesh_resource = ebt_mesh_path_;
        marker_transform.scale.x = 1;
        marker_transform.scale.y = 1;
        marker_transform.scale.z = 1;
        marker_transform.color.r = 0.0;
        marker_transform.color.g = 1.0;
        marker_transform.color.b = 0.0;
        marker_transform.color.a = 1.0;

        // Set the marker's pose
        marker_transform.action = visualization_msgs::Marker::ADD;
        marker_transform.pose.position.x = pose(0,3);
        marker_transform.pose.position.y = pose(1,3);
        marker_transform.pose.position.z = pose(2,3);
        marker_transform.pose.orientation.x = q.x();
        marker_transform.pose.orientation.y = q.y();
        marker_transform.pose.orientation.z = q.z();
        marker_transform.pose.orientation.w = q.w();

        // Add the marker to marker array message
        marker_transforms.markers.push_back(marker_transform);

        // Fill in Object detection.
        object_tracking_2d_ros::ObjectDetection object_det;
        object_det.header = marker_transform.header;
        object_det.id = marker_transform.id;
        object_det.ns = marker_transform.ns;
        object_det.pose.pose = marker_transform.pose;
        object_det.pose.covariance = cov_;
        object_det.good = d.good;

        // Add the detection to detection array message
        object_detections.detections.push_back(object_det);
    }

    // Publish the marker and detection messages
    marker_publisher_.publish(marker_transforms);
    ebt_publisher_.publish(object_detections);
    */

}

void ConnectCallback(const ros::SingleSubscriberPublisher& info)
{
    // Check for subscribers.
    uint32_t subscribers = detection_publisher_.getNumSubscribers();
    ROS_DEBUG("Subscription detected! (%d subscribers)", subscribers);

    if(subscribers && !running_)
    {
        ROS_DEBUG("New Subscribers, Connecting to Input Image Topic.");
        ros::TransportHints ros_transport_hints(ros::TransportHints().tcpNoDelay());
        image_transport::TransportHints image_transport_hint(image_transport::TransportHints(
                                                                 "raw", ros_transport_hints, (*node_),
                                                                 "image_transport"));

        image_subscriber = (*image_).subscribe(
                    DEFAULT_IMAGE_TOPIC, 1, &ImageCallback,
                    image_transport_hint);
        info_subscriber = (*node_).subscribe(
                    DEFAULT_CAMERA_INFO_TOPIC, 10, &InfoCallback);
        running_ = true;
    }
}

void DisconnectHandler()
{
}

void DisconnectCallback(const ros::SingleSubscriberPublisher& info)
{
    // Check for subscribers.
    uint32_t subscribers = detection_publisher_.getNumSubscribers();
    ROS_DEBUG("Unsubscription detected! (%d subscribers)", subscribers);

    if(!subscribers && running_)
    {
        ROS_DEBUG("No Subscribers, Disconnecting from Input Image Topic.");
        image_subscriber.shutdown();
        info_subscriber.shutdown();
        running_ = false;
    }
}

void GetParameterValues()
{
    // Load node-wide configuration values.
//    node_->param ("ebt_tracker_type", ebt_tracker_type_, std::string("irls"));
}

//void ParameterCallback(ros_caffe::ros_caffe_rosConfig &config, uint32_t level) {
////    ROS_INFO("Reconfigure Request");
//////    ebt_min_keypoint_ = config.ebt_min_keypoint;
//}

void SetupPublisher()
{
    // Add callbacks
    ros::SubscriberStatusCallback connect_callback = &ConnectCallback;
    ros::SubscriberStatusCallback disconnect_callback = &DisconnectCallback;

    // Publisher
//    detection_publisher_ = node_->advertise<object_tracking_2d_ros::ObjectDetections>(
//                DEFAULT_DETECTIONS_TOPIC, 1, connect_callback, disconnect_callback);
}

void SetupSubscriber()
{
}

void InitializeClassifier()
{

}

void InitializeROSNode(int argc, char **argv)
{
    ros::init(argc, argv, "ros_caffe");
    node_ =  boost::make_shared<ros::NodeHandle>("~");
    image_ = boost::make_shared<image_transport::ImageTransport>(*node_);

}

int main(int argc, char **argv)
{
    // Initialize Node
    InitializeROSNode(argc,argv);
    GetParameterValues();
    SetupPublisher();
    SetupSubscriber();
    InitializeClassifier();

    // Start Node
    ROS_INFO("ros_caffe node started.");
    running_ = false;
    has_camera_info_ = false;

//    dynamic_reconfigure::Server<Config> server;
//    dynamic_reconfigure::Server<Config>::CallbackType f;
//    f = boost::bind(&ParameterCallback, _1, _2);
//    server.setCallback(f);

    // start processing callbacks
    while(ros::ok() && !quit_)
    {
        ros::spinOnce();
    }
    ROS_INFO("ros_caffe node stopped.");

    return EXIT_SUCCESS;
}
