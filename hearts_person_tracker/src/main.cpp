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

class ImageRegion
{
Mat _region; 
double _startX;
double _startY; 
double _endX;
double _endY;
string _name;
double _distanceFromCenter;
double _indexX;
double _indexY;
bool _match;

public:
    ImageRegion()
    {
    }
    
    ImageRegion(const Mat& region, double startX, double startY, double endX, double endY, string name, double distanceFromCenter, double indexX, double indexY, bool match)
    {
        _region = region;
        _startX = startX;
        _startY = startY;
        _endX = endX;
        _endY = endY;
        _name = name;
        _distanceFromCenter = distanceFromCenter;
        _indexX = indexX;
        _indexY = indexY;
        _match = match;
    }
        
    Mat& getRegion()
    {
        return _region;
    }
    
    double getStartX()
    {
        return _startX;
    }
    
    double getStartY()
    {
        return _startY;
    }
    
    double getEndX()
    {
        return _endX;
    }
    
    double getEndY()
    {
        return _endY;
    }
    
    string getName()
    {
        return _name;
    }
    
    double getDistanceFromCenter() const
    {
        return _distanceFromCenter;
    }
    
    double getIndexX()
    {
        return _indexX;
    }
    
    double getIndexY()
    {
        return _indexY;
    }
    
    bool getMatch()
    {
        return _match;
    }
       
    bool operator<(const ImageRegion& rhs) const 
    {
       return getDistanceFromCenter() < rhs.getDistanceFromCenter();
    }
};
     
class Params
{
    double _b;
    double _d;
    double _g;
    double _r;
    
public:
    Params(double b, double d, double g, double r)
    {
        _b = b;
        _d = d;
        _g = g;
        _r = r;
    }
    
    double getB()
    {
        return _b;
    }
    
    double getD()
    {
        return _d;
    }
    
    double getG()
    {
        return _g;
    }
    
    double getR()
    {
        return _r;
    }
};

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber dTopicSub_;
  image_transport::Subscriber rgbTopicSub_;
  
  image_transport::Publisher personImgPub_;
  ros::Publisher personXPub_;
  ros::Publisher headPub_;
  
  Ptr<FaceRecognizer> model_;
  
  Mat _d;
  Mat _rgb;
  bool _started;
  
  double _bRegionAvgVal;
  double _dRegionAvgVal; 
  double _gRegionAvgVal; 
  double _rRegionAvgVal;
  
  double _bRegionStdDev;
  double _gRegionStdDev;
  double _rRegionStdDev;
  
  double _h_margin;
  double _v_margin;
  double _n_h_regions;
  double _n_v_regions;
  
  double _threshold;
  double _threshold2;
  double _threshold3;
  
  deque<Vec3d> _prev;
  deque<double> _prevX;
  deque<double> _prevTheta;
  int _nPrev;
  double angleRads_;
    ros::Subscriber sub;
    double _h_2;
    double _w_2;
    
    bool _paused;
public:
    ImageConverter();
    ~ImageConverter();
    bool process(Vec3d& curr);
    bool train(Mat& d, Mat& rgb, double& dRegionAvgVal, double& rRegionAvgVal, double& gRegionAvgVal, double& bRegionAvgVal, double& rRegionStdDev, double& gRegionStdDev, double& bRegionStdDev);
    
private:
    string toString(double a);
    void dTopicCb(const sensor_msgs::ImageConstPtr& msg);
    void rgbTopicCb(const sensor_msgs::ImageConstPtr& msg);
    void jointStatesTopicCb(const sensor_msgs::JointState::ConstPtr& msg);
    bool isMatch(double bRegionAvgVal, double gRegionAvgVal, double rRegionAvgVal, double bRegionStdDev, double gRegionStdDev, double rRegionStdDev);
    bool isMatch2(double bRegionAvgVal, double gRegionAvgVal, double rRegionAvgVal, double bRegionStdDev, double gRegionStdDev, double rRegionStdDev);
    void rotateHead(double angleRads);
    bool Start(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);
    bool Stop(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res);
    bool Pause(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);
    bool Unpause(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);
};

bool ImageConverter::Pause(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
{
    res.success = _paused = true;
    return true;
}

bool ImageConverter::Unpause(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
{
    _paused = false;
    res.success = true;
    return true;
}

bool ImageConverter::Start(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
{
    cout << "Starting..." << endl;
    
    _started = train(_d, _rgb, _dRegionAvgVal, _rRegionAvgVal, _gRegionAvgVal, _bRegionAvgVal, _rRegionStdDev, _gRegionStdDev, _bRegionStdDev);
            
    if (_started)
    {
        cout << "Started." << endl;
        
        Size size = _rgb.size();
        _h_2 = size.height/2;
        _w_2 = size.width/2;
        
        cout << "TRAINED: _bRegionAvgVal: " << _bRegionAvgVal << ", _gRegionAvgVal: " << _gRegionAvgVal << ", _rRegionAvgVal: " << _rRegionAvgVal << ", _bRegionStdDev: " << _bRegionStdDev << ", _gRegionStdDev: " << _gRegionStdDev << ", _rRegionStdDev: " << _rRegionStdDev << endl;
    }
    else
        cout << "Failed to start." << endl;
    
    res.success = _started;
    return true;
}

bool ImageConverter::Stop(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res)
{
    _started = false;
    return true;
}

// state machine with states: started, stopped

ImageConverter::ImageConverter()
    : it_(nh_)
{
    _h_margin = 20;
    _v_margin = 20;
    _n_h_regions = 15;
    _n_v_regions = 11;
    _threshold = 20;
    _threshold2 = 15;
    _threshold3 = 15;
    _nPrev = 3;
    _started = false;
        
    sub = nh_.subscribe("joint_states", 10, &ImageConverter::jointStatesTopicCb, this);
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
        
    personImgPub_ = it_.advertise("/image_converter/output_video", 1);
    personXPub_ = nh_.advertise<std_msgs::Float32>("/hearts/person/x", 1000);

    headPub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/head_controller/command", 1);

    // Load the cascades
    if (!face_cascade.load(faceCascadePath))
    { 
        printf("--(!)Error loading face cascade\n"); 
        return; 
    }

    cv::namedWindow(D_OPENCV_WINDOW);
    cv::namedWindow(RGB_OPENCV_WINDOW);
    
    ros::ServiceServer startService = nh_.advertiseService("start_person_tracking", &ImageConverter::Start, this);
    ros::ServiceServer pauseService = nh_.advertiseService("pause_person_tracking", &ImageConverter::Pause, this);
    ros::ServiceServer unpauseService = nh_.advertiseService("unpause_person_tracking", &ImageConverter::Unpause, this);
    ros::ServiceServer stopService = nh_.advertiseService("stop_person_tracking", &ImageConverter::Stop, this);
    
    ros::Publisher pubVel = nh_.advertise<geometry_msgs::Twist>("/key_vel", 10);
    
    ros::Rate loopRate(10); // 1
    
    double angle_rad = 0;
    double angle_inc_rad = M_PI / 10;

    while (ros::ok())
    {
        if (_started && !_paused)
        {                          
            Vec3d curr;
            if (process(curr))
            {
                //cout << curr << endl;
            
                circle(_rgb, Point(320, 240), 3, Scalar(0,255,0));
                double a1 = curr[0];
                double a2 = curr[1];
                double h = curr[2];
                //cout << "o: " << a1 << ", h: " << h << endl;
                double theta = asin(a1/h);
                //cout << "theta: " << (theta / (2*M_PI)) * 360 << " degrees" << endl;
                /*
                // TODO: need to get this working!
                double theta_lr_rad_inc = -((M_PI/2) - acos(a1/h)); 
                double theta_lr_rad = angleRads_ + theta_lr_rad_inc;
          
                double theta_ud_rad = (M_PI/2) - acos(a2/h);
                
                if (!isnan(theta_lr_rad))
                {
                    if (theta_lr_rad < -(M_PI/2))
                        theta_lr_rad = -(M_PI/2);
                        
                    if (theta_lr_rad > (M_PI/2))
                        theta_lr_rad = (M_PI/2);
                    
                    cout << theta_lr_rad << endl;
                    rotateHead(theta_lr_rad);
                }
                */
                
                // signal smoothing using moving average algorithm (other options e.g. kalman filter may perform better) 
                /*
                if (_prev.size() > _nPrev)
                    _prev.pop_front();
                    
                _prev.push_back(curr);
                
                double sum0 = 0;
                double sum1 = 0;
                double sum2 = 0;
                
                for (int i = 0; i < _prev.size(); i++)
                {
                    sum0 += _prev[i][0];
                    sum1 += _prev[i][1];
                    sum2 += _prev[i][2];
                }
                
                double avg0 = sum0 / _prev.size();            
                double avg1 = sum1 / _prev.size();
                double avg2 = sum2 / _prev.size();
                               
                circle(_rgb, Point(avg0, avg1), 3, Scalar(0,0,255));
                */
                cv::imshow("MASK3", _rgb);
                
        
                //double theta_lr_rad_inc = -((M_PI/2) - acos(a1/h)); 
                //double theta_lr_rad = angleRads_ + theta_lr_rad_inc;
          
                geometry_msgs::Twist msg;
                
                double thetaLimit = M_PI/180; // 1 degree 
                double xLimit = 0.05; // 5 cm
                                
                if (!isnan(theta))
                {
                    if (_prevTheta.size() > _nPrev)
                        _prevTheta.pop_front();

                    if (theta < -thetaLimit)
                        _prevTheta.push_back(+0.3); 
                    else if (theta > +thetaLimit)
                        _prevTheta.push_back(-0.3);
                    else 
                        _prevTheta.push_back(0);

                    if (_prevX.size() > _nPrev)
                        _prevX.pop_front();
                        
                    double x = _dRegionAvgVal - h;
                    
                    cout << "distance delta : " << x << endl;
                    
                    if (x < -xLimit)
                        _prevX.push_back(+0.5);
                    else if (x > +xLimit)
                        _prevX.push_back(-0.5); 
                    else
                        _prevX.push_back(0);
                        
                    double sum0 = 0;
                
                    for (int i = 0; i < _prevTheta.size(); i++)
                        sum0 += _prevTheta[i];
                    
                    double sum1 = 0;
                    
                    for (int i = 0; i < _prevX.size(); i++)
                        sum1 += _prevX[i];

                    msg.linear.x = sum1 / _prevX.size();            
                    msg.angular.z = sum0 / _prevTheta.size();
                }
                
                pubVel.publish(msg);                
            }
            else
            {
                // stop!
                geometry_msgs::Twist msg;
                msg.linear.x = 0;
                msg.angular.z = 0;
                pubVel.publish(msg);
            }
        }
        else
        {
            // stop!
            geometry_msgs::Twist msg1;
            msg1.linear.x = 0;
            msg1.angular.z = 0;
            pubVel.publish(msg1);
        }
        
        ros::spinOnce();
        loopRate.sleep();
    }
}

// base control should a) attempt to minimise angle between head and base

ImageConverter::~ImageConverter()
{
    destroyWindow(D_OPENCV_WINDOW);
    destroyWindow(RGB_OPENCV_WINDOW);
    image_transport::Publisher personImgPub_;
}

bool ImageConverter::isMatch(double bRegionAvgVal, double gRegionAvgVal, double rRegionAvgVal, double bRegionStdDev, double gRegionStdDev, double rRegionStdDev)
{
    return sqrt(pow(bRegionAvgVal - _bRegionAvgVal, 2) + pow(gRegionAvgVal - _gRegionAvgVal, 2) + pow(rRegionAvgVal - _rRegionAvgVal, 2) + pow(bRegionStdDev - _bRegionStdDev, 2) + pow(gRegionStdDev - _gRegionStdDev, 2) + pow(rRegionStdDev - _rRegionStdDev, 2)) < _threshold;
}
 
bool ImageConverter::isMatch2(double bRegionAvgVal, double gRegionAvgVal, double rRegionAvgVal, double bRegionStdDev, double gRegionStdDev, double rRegionStdDev)
{
    return (abs(bRegionAvgVal - _bRegionAvgVal) < _threshold2) && (abs(gRegionAvgVal - _gRegionAvgVal) < _threshold2) && (abs(rRegionAvgVal - _rRegionAvgVal) < _threshold2) && (abs(bRegionStdDev - _bRegionStdDev) < _threshold3) && (abs(gRegionStdDev - _gRegionStdDev) < _threshold3) && (abs(rRegionStdDev - _rRegionStdDev) < _threshold3);
} 
  
bool ImageConverter::process(Vec3d& curr)
{ 
    Size size = _rgb.size();
    Mat mask = Mat(size, CV_8UC1, 255);
    Mat img1 = _rgb.clone();
    Mat img2 = _rgb.clone();
    
    double h = size.height;
    double w = size.width;
    double h_2 = h / 2;
    double w_2 = w / 2;    

    double region_h = (h - (2*_v_margin)) / _n_v_regions;
    double region_w = (w - (2*_h_margin)) / _n_h_regions;    
    double region_h_2 = region_h / 2;
    double region_w_2 = region_w / 2;
    
    for (int i = 0; i < _n_v_regions; i++)
    {
        for (int j = 0; j < _n_h_regions; j++)
        {    
            double startY = (region_h * i) + _v_margin;
            double startX = (region_w * j) + _h_margin;
            double endY = startY + region_h;
            double endX = startX + region_w;
            double centerY = startY + region_h_2;
            double centerX = startX + region_w_2;
            double distanceFromCenter = sqrt(pow(h_2 - centerY, 2) + pow(w_2 - centerX, 2));
            double indexX = j;
            double indexY = i;                
            string name = "(" + toString(indexY) + "," + toString(indexX) + ")";
            
            //putText(_rgb, name, Point(centerX, centerY), FONT_HERSHEY_PLAIN, 0.5, Scalar(255,0,0));
            
            Rect rect;
            rect.x = startX;
            rect.y = startY;
            rect.width = endX - startX;
            rect.height = endY - startY;
            Mat region = _rgb(rect);
            
            Mat channels[3];
            split(region, channels);
            
            Scalar mean0, mean1, mean2, stdDev0, stdDev1, stdDev2;
            meanStdDev(channels[0], mean0, stdDev0); 
            meanStdDev(channels[1], mean1, stdDev1); 
            meanStdDev(channels[2], mean2, stdDev2);
            
            double bRegionAvgVal = mean0.val[0];
            double gRegionAvgVal = mean1.val[0];
            double rRegionAvgVal = mean2.val[0];
            
            double bRegionStdDev = stdDev0.val[0];
            double gRegionStdDev = stdDev1.val[0];
            double rRegionStdDev = stdDev2.val[0];
            
            bool match = false;
            if (isMatch2(bRegionAvgVal, gRegionAvgVal, rRegionAvgVal, bRegionStdDev, gRegionStdDev, rRegionStdDev))
            {
                match = true;
                rectangle(mask, rect, Scalar(0, 0, 0), CV_FILLED);
            }

            rectangle(img1, rect, Scalar(255,0,0), match ? CV_FILLED : 1);                    
        }
    }
        
    int n_off_v_regions = _n_v_regions - 1;
    int n_off_h_regions = _n_h_regions - 1;
    
    for (int i = 0; i < n_off_v_regions; i++)
    {
        for (int j = 0; j < n_off_h_regions; j++)
        {   
            double startY = (region_h * i) + _v_margin + region_h_2;
            double startX = (region_w * j) + _h_margin + region_w_2;
            double endY = startY + region_h;
            double endX = startX + region_w;
            double centerY = startY + region_h_2;
            double centerX = startX + region_w_2;
            double distanceFromCenter = sqrt(pow(h_2 - centerY, 2) + pow(w_2 - centerX, 2));
            double indexX = j + 0.5;
            double indexY = i + 0.5; 
            string name = "(" + toString(indexY) + "," + toString(indexX) + ")";
            
            //putText(_rgb, name, Point(centerX, centerY), FONT_HERSHEY_PLAIN, 0.5, Scalar(0,0,255));
            
            Rect rect;
            rect.x = startX;
            rect.y = startY;
            rect.width = endX - startX;
            rect.height = endY - startY;
            Mat region = _rgb(rect);
            
            Mat channels[3];
            split(region, channels);
            
            Scalar mean0, mean1, mean2, stdDev0, stdDev1, stdDev2;
            meanStdDev(channels[0], mean0, stdDev0); 
            meanStdDev(channels[1], mean1, stdDev1); 
            meanStdDev(channels[2], mean2, stdDev2);
            
            double bRegionAvgVal = mean0.val[0];
            double gRegionAvgVal = mean1.val[0];
            double rRegionAvgVal = mean2.val[0];
            
            double bRegionStdDev = stdDev0.val[0];
            double gRegionStdDev = stdDev1.val[0];
            double rRegionStdDev = stdDev2.val[0];
            
            bool match = false;
            if (isMatch2(bRegionAvgVal, gRegionAvgVal, rRegionAvgVal, bRegionStdDev, gRegionStdDev, rRegionStdDev))
            {
                match = true;
                rectangle(mask, rect, Scalar(0, 0, 0), CV_FILLED);
            }

            rectangle(img1, rect, Scalar(0,0,255), match ? CV_FILLED : 1);
        }
    }
    
    Mat mask2 = mask.clone();
           
    // grow adjacent image regions, then return enclosing rect
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(mask2, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0)); // 
    
    double largestArea = 0;
    int largestContourIndex = -1;
    
    for (int i = 0; i < contours.size(); i++) // iterate through each contour. 
    {
        // exclude outermost contour which has no parents
        if (hierarchy[i][3] < 0)
            continue;
            
        double area = contourArea(contours[i], false);  //  Find the area of contour
        if (largestArea < area)
        {
            largestArea = area;
            largestContourIndex = i;                //Store the index of largest contour
        }
    }
  
    //for (size_t i = 0; i < contours.size(); i++)
    //{
    //    drawContours( img2, contours, (int)i, Scalar(255, 0, 0), 2, 8, hierarchy, 0, Point() );
    //}
    
    cv::imshow("MASK0", mask);
    cv::imshow("MASK1", img1);
    cv::waitKey(3);
    
    if (largestContourIndex > -1)
    {
        Mat invertedMask;
        bitwise_not(mask, invertedMask);
    
        // Chris's idea - recalculate training parameters on each image frame
        
        cv::imshow("INVERTED MASK", invertedMask);
        
        Mat newChannels[3];
        split(_rgb, newChannels);
        
        Scalar newMean0, newMean1, newMean2, newStdDev0, newStdDev1, newStdDev2;
        meanStdDev(newChannels[0], newMean0, newStdDev0, invertedMask); 
        meanStdDev(newChannels[1], newMean1, newStdDev1, invertedMask); 
        meanStdDev(newChannels[2], newMean2, newStdDev2, invertedMask);
                
        _bRegionAvgVal = newMean0.val[0];
        _gRegionAvgVal = newMean1.val[0];
        _rRegionAvgVal = newMean2.val[0];
                
        _bRegionStdDev = newStdDev0.val[0];
        _gRegionStdDev = newStdDev1.val[0];
        _rRegionStdDev = newStdDev2.val[0];
        
        //cout << "_bRegionAvgVal: " << _bRegionAvgVal << ", _gRegionAvgVal: " << _gRegionAvgVal << ", _rRegionAvgVal: " << _rRegionAvgVal << ", _bRegionStdDev: " << _bRegionStdDev << ", _gRegionStdDev: " << _gRegionStdDev << ", _rRegionStdDev: " << _rRegionStdDev << endl;
    
        Scalar newMeanD = mean(_d, invertedMask);
        double d = newMeanD.val[0];
        
        Rect largestRect = boundingRect(contours[largestContourIndex]); // Find the bounding rectangle for biggest contour
        Point center = (largestRect.br() + largestRect.tl())*0.5;
        Vec3d curr_n;
        curr_n = Vec3d(center.x, center.y, d);
        
        
        
        double z_m = (d / 10000);//0.10131);
        double x_m = ((center.x - w_2) / 540.0) * z_m;
        double y_m = ((center.y - h_2) / 540.0) * z_m;
        
        cout << "distance: " << z_m << " m" << endl;
        
        curr = Vec3d(x_m, y_m, z_m);
        circle(img2, center, 3, Scalar(0,255,0));
        cv::imshow("MASK2", img2);
        cv::waitKey(3);
        
        //double centerX = largestRect.x + largestRect.width/2;
        //double centerY = largestRect.y + largestRect.height/2;
        
        //rectangle(img2, largestRect, Scalar(0, 255, 0), 1);
        return true;
    }
    

    
    
    //cv::imshow("MASK3", drawing);
    
    
    //cv::imshow("MASK", _rgb);
    //cv::waitKey(3);
    //bitwise_and(_d, mask, _d);
    
    return false;
}

bool ImageConverter::train(Mat& d, Mat& rgb, double& dRegionAvgVal, double& rRegionAvgVal, double& gRegionAvgVal, double& bRegionAvgVal, double& rRegionStdDev, double& gRegionStdDev, double& bRegionStdDev)
{    
    if (d.empty() || rgb.empty())
         return false;
                 
    // TODO: need to calculate mean distance as well! i actually need to call into a function each time depth or image frame changes
    
    Mat frame = rgb.clone();
    
    /*
    Mat frame_gray;
    cvtColor(rgb, frame_gray, COLOR_BGR2GRAY);

    //-- Detect faces
    std::vector<Rect> faces;
    face_cascade.detectMultiScale(frame_gray, faces, 1.2, 4, 0 | CASCADE_SCALE_IMAGE, Size(50, 50)); // this is face detection, MIN_FACE_SIZE

    if ((faces.size() > 3) || (faces.size() < 1)) // skip if there is no face or there are more than 3 faces
        return false;
    
    int largestIndex = 0;
    int faceSize = faces[0].width;

    for (size_t i = 0; i < faces.size(); i++) // this is to find the largest face and process it only
    {
        //Point center(faces[i].x + faces[i].width / 2, faces[i].y + faces[i].height / 2);
        //ellipse(frame, center, Size(faces[i].width / 2, faces[i].height / 2), 0, 0, 360, Scalar(255, 0, 255), 4, 8, 0);
        Mat faceROI = frame_gray(faces[i]);
        if (faces[i].width > faceSize)
        {
            largestIndex = i;
        }
    }

    Rect largestFaceBox = faces[largestIndex];
    int label;
    double confidence;
    Mat oneFace = frame_gray(faces[largestIndex]);
    Mat oneFaceSmall;
    Size imgsize(200,200);
    resize(oneFace, oneFaceSmall, imgsize);
    
    // sampling a region for uniform recognition. Variables: region is on the belly
    Rect region;
    region.x = largestFaceBox.x;
    region.y = largestFaceBox.y + 1.8 * largestFaceBox.width;
    region.width = largestFaceBox.width;
    region.height = 1.6 * largestFaceBox.height;

    rectangle(frame, largestFaceBox, BLUE);
    rectangle(frame, region, GREEN);
    //cout << frame.rows << endl << frame.cols << endl;

    if ((region.x + region.width > rgb.cols) || (region.y + region.height > rgb.rows)) // make sure we don't process the regions when they're out of frame
        return false;
*/
// 640 x 480
    Rect region;
    region.x = 220;
    region.y = 230;
    region.width = 200;
    region.height = 200;
    rectangle(frame, region, BLUE);
    
    Mat dRegion = d(region);
    Scalar dRegionAvg = mean(dRegion);
    //dRegionAvgVal = dRegionAvg.val[0] / 10000; //0.10131; // double 
    dRegionAvgVal = 0.001; //0.10131; // double 
    //cout << "avg. depth = " << dRegionAvgVal << endl;
    
    Mat rgbRegion = rgb(region);
    Mat rgbRegionChannels[3];
    split(rgbRegion, rgbRegionChannels);
    
    Scalar mean0, mean1, mean2, stdDev0, stdDev1, stdDev2;
    meanStdDev(rgbRegionChannels[0], mean0, stdDev0); 
    meanStdDev(rgbRegionChannels[1], mean1, stdDev1); 
    meanStdDev(rgbRegionChannels[2], mean2, stdDev2);
    
    bRegionAvgVal = mean0.val[0];
    gRegionAvgVal = mean1.val[0];
    rRegionAvgVal = mean2.val[0];
    
    bRegionStdDev = stdDev0.val[0];
    gRegionStdDev = stdDev1.val[0];
    rRegionStdDev = stdDev2.val[0];
    
    //cout << "avg. red = " << rRegionAvgVal << endl;
    //cout << "avg. green = " << gRegionAvgVal << endl;
    //cout << "avg. blue = " << bRegionAvgVal << endl << endl;
    
    cv::imshow(OUT_OPENCV_WINDOW, frame);
    cv::waitKey(3);
    
    // Update GUI Window
    
    // Output modified video stream
    //personImgPub_.publish(cv_ptr->toImageMsg());
    
    return true;
}

void ImageConverter::rotateHead(double angleRads)
{
    trajectory_msgs::JointTrajectory headTrajectory;
    headTrajectory.joint_names.resize(2);
    headTrajectory.joint_names[0] = "head_2_joint";
    headTrajectory.joint_names[1] = "head_1_joint";
    
    headTrajectory.points.resize(1);
    
    double headUd = 0;
    double headLr = angleRads;
    
    headTrajectory.points[0].positions.resize(2);
    headTrajectory.points[0].positions[0] = headUd;
    headTrajectory.points[0].positions[1] = headLr;
    
    headTrajectory.points[0].velocities.resize(2);
    headTrajectory.points[0].accelerations.resize(2);
    headTrajectory.points[0].effort.resize(2);
    headTrajectory.points[0].time_from_start = ros::Duration(2.0, 0.0);
    
    headPub_.publish(headTrajectory);
   
    angleRads_ = std::numeric_limits<double>::max();
   
    //ros::Subscriber sub;
    //sub = nh_.subscribe("joint_states", 10, &ImageConverter::jointStatesTopicCb, this);
    
    ros::Rate rate(10);
    
    while (ros::ok())
    {
        if ((angleRads_ < std::numeric_limits<double>::max()) && 
            (abs(angleRads_ - angleRads) < 0.1))
        {
           break;
        }
        
        ros::spinOnce();
        rate.sleep();
    }

    //sub.shutdown();
}

void ImageConverter::jointStatesTopicCb(const sensor_msgs::JointState::ConstPtr& msg)
{
    for (int i = 0; i < msg->name.size(); i++)
    {
        if (msg->name[i] == "head_1_joint")
        {
            angleRads_ = msg->position[i];
            return;
        }
    }
    
    angleRads_ = std::numeric_limits<double>::max();
}

string ImageConverter::toString(double a)
{
    stringstream ss;
    ss << a;
    return ss.str();
}    
  
void ImageConverter::dTopicCb(const sensor_msgs::ImageConstPtr& msg)
{
// 1m = 10000
// 2m = 20000
    try
    {
        cv_bridge::CvImagePtr d_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        Mat d = d_ptr->image;
        //cv::normalize(d, _d, 1, 0, cv::NORM_MINMAX);
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

