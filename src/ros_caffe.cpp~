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
    cv::Mat img = subscribed_ptr->image;

    // Run the classifier
    std::vector<Prediction> predictions = classifier_->Classify(img);

    // Publish the predictions
    PublishPredictions(predictions);
}

void PublishPredictions(const std::vector<Prediction>& predictions)  {
    std_msgs::String msg;
    std::stringstream ss;
    for (size_t i = 0; i < predictions.size(); ++i) {
        Prediction p = predictions[i];
        ss << "[" << p.second << " - " << p.first << "]" << std::endl;
    }
    msg.data = ss.str();
    predictions_publisher_.publish(msg);
}

void ConnectCallback(const ros::SingleSubscriberPublisher& info)
{
    // Check for subscribers.
    uint32_t subscribers = predictions_publisher_.getNumSubscribers();
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

void DisconnectCallback(const ros::SingleSubscriberPublisher& info)
{
    // Check for subscribers.
    uint32_t subscribers = predictions_publisher_.getNumSubscribers();
    ROS_DEBUG("Unsubscription detected! (%d subscribers)", subscribers);

    if(!subscribers && running_)
    {
        ROS_DEBUG("No Subscribers, Disconnecting from Input Image Topic.");
        image_subscriber.shutdown();
        info_subscriber.shutdown();
        running_ = false;
    }
}

void DisconnectHandler()
{
}

void GetParameterValues()
{
    // Load node-wide configuration values.
    node_->param ("model_path",     model_path_, std::string(""));
    node_->param ("weights_path",   weights_path_, std::string(""));
    node_->param ("mean_file",      mean_file_, std::string(""));
    node_->param ("label_file",     label_file_, std::string(""));

    node_->param ("test_image",     test_image_, false);
    node_->param ("image_path",     image_path_, std::string(""));
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
    predictions_publisher_ = node_->advertise<std_msgs::String>(
                DEFAULT_PREDICTIONS_TOPIC, 1, connect_callback, disconnect_callback);
}

void InitializeClassifier()
{
    classifier_ = new Classifier(model_path_, weights_path_, mean_file_, label_file_);
    if (test_image_){
        cv::Mat img = cv::imread(image_path_, -1);
        ROS_INFO("Predicting Test Image");
        std::vector<Prediction> predictions = classifier_->Classify(img);
        /* Print the top N predictions. */
        for (size_t i = 0; i < predictions.size(); ++i) {
            Prediction p = predictions[i];
            ROS_INFO_STREAM("Prediction: " << std::fixed << std::setprecision(4) << p.second << " - \"" << p.first << "\"" << std::endl);
        }
        PublishPredictions(predictions);
    }
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

    // Delete stuff
    delete classifier_;

    return EXIT_SUCCESS;
}
