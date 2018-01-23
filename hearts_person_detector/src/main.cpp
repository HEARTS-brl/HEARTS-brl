#define _USE_MATH_DEFINES

#include <limits>
#include <cstddef>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <stdlib.h>
#include "opencv2/core/version.hpp"
#include <ros/ros.h>
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include "trajectory_msgs/JointTrajectory.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/JointState.h>
#include <opencv2/core/core.hpp>
// #include <opencv2/face.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include "macros.hpp"
#include "math.h"
#include "std_srvs/Trigger.h"
#include "std_srvs/Empty.h"


using namespace cv;
using namespace ros;
using namespace std;

// using namespace cv::face;

/** Global variables */
CascadeClassifier face_cascade;

static const string D_TOPIC_PARAMETER = "d_topic";

static const string D_OPENCV_WINDOW = "D";

static const string FACE_CASCADE_PATH_PARAMETER = "face_cascade_path";

static const string OUT_OPENCV_WINDOW = "OUT";

static const string RGB_OPENCV_WINDOW = "RGB";

static const string RGB_TOPIC_PARAMETER = "rgb_topic";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber dTopicSub_;
  image_transport::Subscriber rgbTopicSub_;
  
  Ptr<FaceRecognizer> model_;
  
  Mat _d;
  Mat _rgb;
  
public:
    ImageConverter();
    ~ImageConverter();
    
private:
    void dTopicCb(const sensor_msgs::ImageConstPtr& msg);
    void rgbTopicCb(const sensor_msgs::ImageConstPtr& msg);
    string toString(double a);
    bool hasFace(Mat& d, Mat& rgb);
    bool hasDoor(Mat& d, Mat& rgb);
    bool Start(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);
};

bool ImageConverter::Start(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
{
    cout << "Processing request..." << endl;
    
    if (hasDoor(_d, _rgb))
        res.message = "2";
    else if (hasFace(_d, _rgb))
        res.message = "1";
    else
        res.message = "3";
    
    cout << "Processed request." << endl;
    
    res.success = true;
    return true;
}

// state machine with states: started, stopped

ImageConverter::ImageConverter()
    : it_(nh_)
{       
    string dTopic;
    if (!nh_.getParam(D_TOPIC_PARAMETER, dTopic))
        cout << "FAILED TO GET PARAMETER: " << D_TOPIC_PARAMETER << endl;
           
    string faceCascadePath;
    if (!nh_.getParam(FACE_CASCADE_PATH_PARAMETER, faceCascadePath))
        cout << "FAILED TO GET PARAMETER: " << FACE_CASCADE_PATH_PARAMETER << endl;
    
    string rgbTopic;
    if (!nh_.getParam(RGB_TOPIC_PARAMETER, rgbTopic))
        cout << "FAILED TO GET PARAMETER: " << RGB_TOPIC_PARAMETER << endl;
        
    dTopicSub_ = it_.subscribe(dTopic, 1, &ImageConverter::dTopicCb, this);
    rgbTopicSub_ = it_.subscribe(rgbTopic, 1, &ImageConverter::rgbTopicCb, this);

    // Load the cascades
    if (!face_cascade.load(faceCascadePath))
    { 
        printf("--(!)Error loading face cascade\n"); 
        return; 
    }

    cv::namedWindow(D_OPENCV_WINDOW);
    cv::namedWindow(RGB_OPENCV_WINDOW);
    
    ros::ServiceServer startService = nh_.advertiseService("start_person_detection", &ImageConverter::Start, this);
    
    ros::spin();
}

// base control should a) attempt to minimise angle between head and base

ImageConverter::~ImageConverter()
{
    destroyWindow(D_OPENCV_WINDOW);
    destroyWindow(RGB_OPENCV_WINDOW);
}

bool ImageConverter::hasDoor(Mat& d, Mat& rgb)
{
    if (d.empty() || rgb.empty())
         return false;
         
    for (int i = 0; i < d.rows; i++)
    {
        for (int j = 0; j < d.cols; j++)
        {
            if (d.at<double>(i, j) > 30000) // 3m
                return false;
        }
    }
    
    return true;
}

bool ImageConverter::hasFace(Mat& d, Mat& rgb)
{    
    if (d.empty() || rgb.empty())
         return false;
                 
    // TODO: need to calculate mean distance as well! i actually need to call into a function each time depth or image frame changes
    
    Mat frame = rgb.clone();
    Mat frame_gray;
    cvtColor(rgb, frame_gray, COLOR_BGR2GRAY);

    //-- Detect faces
    std::vector<Rect> faces;
    face_cascade.detectMultiScale(frame_gray, faces, 1.2, 4, 0 | CASCADE_SCALE_IMAGE, Size(50, 50)); // this is face detection, MIN_FACE_SIZE

    return faces.size() > 0;
}

string ImageConverter::toString(double a)
{
    stringstream ss;
    ss << a;
    return ss.str();
}    
  
void ImageConverter::dTopicCb(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv_bridge::CvImagePtr d_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        Mat d = d_ptr->image;
        _d = d;
        cv::imshow(D_OPENCV_WINDOW, _d);
        cv::waitKey(3);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("dTopicCb: %s", e.what());
        return;
    }
}
    
void ImageConverter::rgbTopicCb(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv_bridge::CvImagePtr rgb_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        _rgb = rgb_ptr->image;
        cv::imshow(RGB_OPENCV_WINDOW, _rgb);
        cv::waitKey(3);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("rgbTopicCb: %s", e.what());
        return;
    }
}
    
int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_converter");
    ImageConverter ic;
    ros::spin();
    return 0;
}

