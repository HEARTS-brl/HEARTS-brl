#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <stdlib.h>
#include "opencv2/core/version.hpp"
#include <ros/ros.h>
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
// #include <opencv2/face.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include "macros.hpp"

using namespace std;
using namespace cv;
// using namespace cv::face;

/** Global variables */
CascadeClassifier face_cascade;

static const string D_TOPIC_PARAMETER = "d_topic";

static const string D_OPENCV_WINDOW = "D";

static const string FACE_CASCADE_PATH_PARAMETER = "face_cascade_path";

static const string RGB_OPENCV_WINDOW = "RGB";

static const string RGB_TOPIC_PARAMETER = "rgb_topic";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber dTopicSub_;
  image_transport::Subscriber rgbTopicSub_;
  
  image_transport::Publisher personImgPub_;
  ros::Publisher personXPub_;
  
  Ptr<FaceRecognizer> model_;
  
private:
//  void pub(std::string image_label)
//  {
//      std_msgs::String msg;
//      std::stringstream ss;
//      ss << image_label;
//      msg.data = ss.str();
//      image_label_pub_.publish(msg);
//  }
  
public:
  ImageConverter()
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
        
    personImgPub_ = it_.advertise("/image_converter/output_video", 1);
    personXPub_ = nh_.advertise<std_msgs::Float32>("/hearts/person/x", 1000);

    // Load the cascades
    if (!face_cascade.load(faceCascadePath))
    { 
      printf("--(!)Error loading face cascade\n"); 
      return; 
    }

    cv::namedWindow(D_OPENCV_WINDOW);
    cv::namedWindow(RGB_OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    destroyWindow(D_OPENCV_WINDOW);
    destroyWindow(RGB_OPENCV_WINDOW);
    image_transport::Publisher personImgPub_;
  }
  
  void dTopicCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
    // TYPE_32FC1
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1); //TYPE_16SC1); // TYPE_16UC1);
      cv::normalize(cv_ptr->image, cv_ptr->image, 1, 0, cv::NORM_MINMAX);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("dTopicCb: %s", e.what());
      return;
    }

    Mat frame = cv_ptr->image;
    
    cv::imshow(D_OPENCV_WINDOW, frame);
    cv::waitKey(3);
  }

  void rgbTopicCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("rgbTopicCb: %s", e.what());
      return;
    }

    Mat frame = cv_ptr->image;
    Mat frame_gray;
    cvtColor(frame, frame_gray, COLOR_BGR2GRAY);

    //-- Detect faces
    std::vector<Rect> faces;
    face_cascade.detectMultiScale(frame_gray, faces, 1.2, 4, 0 | CASCADE_SCALE_IMAGE, Size(50, 50)); // this is face detection, MIN_FACE_SIZE

    if ((faces.size() > 3) || (faces.size() < 1)) // skip if there is no face or there are more than 3 faces
    {
      imshow(RGB_OPENCV_WINDOW, frame);
      int c = waitKey(3);
      if ((char)c == 27) { return; } // escape
      return;
    }

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

    if ((region.x + region.width > frame.cols) || (region.y + region.height > frame.rows)) // make sure we don't process the regions when they're out of frame
    {
      imshow(RGB_OPENCV_WINDOW, frame);
      int c = waitKey(3);
      if ((char)c == 27) { return; } // escape
      return;
    }

    Mat sample = frame(region);

    Mat rgb[3];
    split(sample, rgb);

    Scalar meanVal0, meanVal1, meanVal2;
    Scalar stdDev0, stdDev1, stdDev2;

    // calculate the mean value and the standard deviation in the region, for R, G and B respectively
    cv::meanStdDev(rgb[0], meanVal0, stdDev0); 
    cv::meanStdDev(rgb[1], meanVal1, stdDev1);
    cv::meanStdDev(rgb[2], meanVal2, stdDev2);

    double sdValB = stdDev0.val[0];
    double sdValG = stdDev1.val[0];
    double sdValR = stdDev2.val[0];
    double avgSDVal = (sdValB + sdValG + sdValR) / 3;

    double meanValB = meanVal0.val[0];
    double meanValG = meanVal1.val[0];
    double meanValR = meanVal2.val[0];
    double meanGrey = (meanValB + meanValG + meanValR) / 3;
 
    cout << meanValB << endl << meanValG << endl << meanValR << endl << avgSDVal << endl << endl;
   
    double rg = meanValR / meanValG;
    double rb = meanValR / meanValB;
    double gb = meanValG / meanValB;
    
    cout << rg << endl << rb << endl << gb << endl << endl;
    
    // TODO: need to calculate mean distance as well! i actually need to call into a function each time depth or image frame changes
    
    // Update GUI Window
    cv::imshow(RGB_OPENCV_WINDOW, frame);
    cv::waitKey(3);
    
    // Output modified video stream
    personImgPub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  printf("running!");
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}

