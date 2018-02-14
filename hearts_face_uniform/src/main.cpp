#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <stdlib.h>
#include "opencv2/core/version.hpp"
#include <ros/ros.h>
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
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace cv;
// using namespace cv::face;

/** Global variables */
CascadeClassifier face_cascade;

static const std::string OPENCV_WINDOW = "Image window";

// where the registered data are stored. You can change this directory
static const std::string DATA_PATH_PARAMETER = "data_path";

static const std::string FACE_CASCADE_PATH_PARAMETER = "face_cascade_path";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher image_label_pub_;
  Ptr<FaceRecognizer> model_;
  string last_image_label_;
  
private:
  void pub(std::string image_label)
  {
    //if (last_image_label_ != image_label)
    //{
      //last_image_label_ = image_label;
      std_msgs::String msg;
      std::stringstream ss;
      ss << image_label;
      msg.data = ss.str();
      image_label_pub_.publish(msg);
    //}
  }
  
public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    //image_sub_ = it_.subscribe("/devices/front_door/image", 1, 
    //  &ImageConverter::imageCb, this);
    
    string dataPath;
    if (!nh_.getParam(DATA_PATH_PARAMETER, dataPath))
        cout << "FAILED TO GET PARAMETER: " << DATA_PATH_PARAMETER << " " << dataPath << endl;
    
    string faceCascadePath;
    if (!nh_.getParam(FACE_CASCADE_PATH_PARAMETER, faceCascadePath))
        cout << "FAILED TO GET PARAMETER: " << FACE_CASCADE_PATH_PARAMETER << endl;
    
    image_sub_ = it_.subscribe("/roah_ipcam/image", 1, 
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
    image_label_pub_ = nh_.advertise<std_msgs::String>("/hearts/face/user", 1000);

    vector<Mat> images;
    vector<int> labels;
    int subjectNum = 1;
    // Read in the data. This OpenCV-3.1.0-can fail if no valid
    // input filename is given.
    for (int x = 1; x <= CAPTURE_NUM; x++)
    {
      stringstream ss;
      ss << x;
      std::string str = ss.str();
      std::string imagePath = dataPath + str + ".jpg";

      cout << "loading " << imagePath << "..." << endl;

      Mat image = imread(imagePath, 0);
      
      if (image.size().area() == 0)
          cout << "FAILED TO LOAD IMAGE" << endl;
      else
      {
          Size imgsize(200,200);
          Mat imgSmall;
          resize(image,imgSmall,imgsize);
          images.push_back(imgSmall);
          labels.push_back(subjectNum);
      }
    }

    // model_ = LBPHFaceRecognizer::create(); // face recogniser created
    model_ = createLBPHFaceRecognizer();
    model_->train(images, labels); // train the model

    // Load the cascades
    if (!face_cascade.load(faceCascadePath))
    { 
      printf("--(!)Error loading face cascade\n"); 
      return; 
    }

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    destroyWindow(OPENCV_WINDOW);
  image_transport::Publisher image_pub_;
  }

    double medianMat(cv::Mat Input)
    {    
        Input = Input.reshape(0,1); // spread Input Mat to single row
        std::vector<double> vecFromMat;
        Input.copyTo(vecFromMat); // Copy Input Mat to vector vecFromMat
        std::nth_element(vecFromMat.begin(), vecFromMat.begin() + vecFromMat.size() / 2, vecFromMat.end());
        return vecFromMat[vecFromMat.size() / 2];
    }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    Mat frame = cv_ptr->image;
    
    //Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
/*
  void imageCb(const sensor_msgs::CompressedImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    Mat frame = cv::imdecode(cv::Mat(msg->data),1);
    */
    //cout << "rows_"<<frame.rows << " and cols_"<<frame.cols << endl;
    std::vector<Rect> faces;
    Mat frame_gray;

    cvtColor(frame, frame_gray, COLOR_BGR2GRAY);
    //equalizeHist(frame_gray, frame_gray);

    //-- Detect faces
    face_cascade.detectMultiScale(frame_gray, faces, 1.2, 4, 0 | CASCADE_SCALE_IMAGE, Size(50, 50)); // this is face detection, MIN_FACE_SIZE

    if ((faces.size() > 3) || (faces.size() < 1)) // skip if there is no face or there are more than 3 faces
    {
      imshow(OPENCV_WINDOW, frame);
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
    model_->predict(oneFaceSmall, label, confidence);

    cout << "confi_"<<confidence << endl<< endl;

    if (confidence < CONFIDENCE_THRESH) // set classification confidence, the higher the value, the less confident
    {
      pub("doctor");
      Point posText(largestFaceBox.x, max(1, largestFaceBox.y - 10));
      putText(frame, "Dr Kimble", posText, FONT_HERSHEY_COMPLEX, 1.2, GREEN, 3, 8); // displaying recognised names
      rectangle(frame, largestFaceBox, GREEN);
      imshow(OPENCV_WINDOW, frame);
      int c = waitKey(3);
      if ((char)c == 27) { return; } // escape
      return;
    }

    // todo: only recognise uniform if face can't be detected within n frames

    //cout << confidence << endl;

    Rect region, regionB, regionC; // sampling a region for uniform recognition. 
    
    // region is on the belly
    region.x = largestFaceBox.x;
    region.y = largestFaceBox.y + 1.8 * largestFaceBox.width;
    region.width = 1.4 * largestFaceBox.width;
    region.height = 1.8 * largestFaceBox.height; // 1.6

    // regionB is on the hat
    regionB.x = largestFaceBox.x;
    regionB.y = largestFaceBox.y - round(0.4 * largestFaceBox.width);
    regionB.width = largestFaceBox.width;
    regionB.height = round(0.6 * largestFaceBox.height);

    // regionC is on the icon of the hat
    regionC.x = regionB.x + round(0.4*regionB.width);
    regionC.y = regionB.y + round(0.42*regionB.height);
    regionC.width = round(0.17*regionB.width);
    regionC.height = round(0.17*regionB.height);

    rectangle(frame, largestFaceBox, BLUE);
    rectangle(frame, region, GREEN);
    rectangle(frame, regionB, GREEN);
    rectangle(frame, regionC, RED);

    //cout << frame.rows << endl << frame.cols << endl;

    if ((region.x + region.width > frame.cols) || (region.y + region.height > frame.rows) || (regionB.x < 1) || (regionB.y < 1)) // make sure we don't process the regions when they're out of frame
    {
      imshow(OPENCV_WINDOW, frame);
      int c = waitKey(3);
      if ((char)c == 27) { return; } // escape
      return;
    }

    Mat sample = frame(region);
    Mat sampleB = frame(regionB);
    cvtColor(sampleB, sampleB, COLOR_BGR2GRAY);
    Mat sampleC = frame(regionC);
    cvtColor(sampleC, sampleC, COLOR_BGR2GRAY);

    Mat rgb[3];
    Mat rgbB[3];

    split(sample, rgb);
    split(sampleB, rgbB);

    Scalar meanVal0, meanVal1, meanVal2;//, meanVal3, meanVal4;
    Scalar stdDev0, stdDev1, stdDev2;//, stdDev3, stdDev4;

    cv::meanStdDev(rgb[0], meanVal0, stdDev0); // calculate the mean value and the standard deviation in the first region, for R, G and B respectively
    cv::meanStdDev(rgb[1], meanVal1, stdDev1);
    cv::meanStdDev(rgb[2], meanVal2, stdDev2);
    
    //cv::meanStdDev(sampleB, meanVal3, stdDev3);
    //cv::meanStdDev(sampleC, meanVal4, stdDev4);

    double sdValB = stdDev0.val[0];
    double sdValG = stdDev1.val[0];
    double sdValR = stdDev2.val[0];
    double avgSDVal = (sdValB + sdValG + sdValR) / 3;

    double meanValB = meanVal0.val[0];
    double meanValG = meanVal1.val[0];
    double meanValR = meanVal2.val[0];
    double meanGrey = (meanValB + meanValG + meanValR) / 3;

    double medianB = medianMat(rgb[0]);
    double medianG = medianMat(rgb[1]);
    double medianR = medianMat(rgb[2]);
    
    cout << "medianB: " << medianB << ", medianG: " << medianG << ", medianR: " << medianR << ", sdValB: " << sdValB << ", sdValG: " << sdValG << ", sdValR: " << sdValR << endl;
    
    //double meanValHat = meanVal3.val[0];
    //double meanValHatSmall = meanVal4.val[0];
 
    //cout << "meanValB: " << meanValB << ", meanValG: " << meanValG << ", meanValR: " << meanValR << ", sdValB: " << sdValB << ", sdValG: " << sdValG << ", sdValR: " << sdValR << endl;
 
    Point posText1(faces[largestIndex].x, max(1, faces[largestIndex].y - 10));

    // rules!!!: checking for white colour, checking for yellow-ish colour, checking for hat pattern, is the hat colour similar to the t-shirt colour
    std::string dispName;
   // if ((max(meanValR, meanValG) / min(meanValR, meanValG) < 1.2) && (max(meanValR, meanValB) / min(meanValR, meanValB) > 1.4) && (max(meanValG, meanValB) / min(meanValG, meanValB) > 1.3) && (avgSDVal < 30) && (meanValHat/meanValHatSmall < 0.8))

/*     if ((abs(meanValB - POSTMAN_B_MEAN) < THRESHOLD) &&
        (abs(meanValG - POSTMAN_G_MEAN) < THRESHOLD) &&
        (abs(meanValR - POSTMAN_R_MEAN) < THRESHOLD) &&
        */    
    if ((abs(medianB - POSTMAN_B_MEAN) < THRESHOLD) &&
        (abs(medianG - POSTMAN_G_MEAN) < THRESHOLD) &&
        (abs(medianR - POSTMAN_R_MEAN) < THRESHOLD) &&
        (abs(sdValB - POSTMAN_B_STDEV) < THRESHOLD) &&
        (abs(sdValG - POSTMAN_G_STDEV) < THRESHOLD) &&
        (abs(sdValR - POSTMAN_R_STDEV) < THRESHOLD))
    {
        pub("postman");
        ROS_INFO("published postman");
        dispName = "POST MAN";
        putText(frame, dispName, posText1, FONT_HERSHEY_COMPLEX, 1.2, AMBER, 2, 8); // displaying recognised names
    }  
    else if ((abs(medianB - DELIMAN_B_MEAN) < THRESHOLD) &&
        (abs(medianG - DELIMAN_G_MEAN) < THRESHOLD) &&
        (abs(medianR - DELIMAN_R_MEAN) < THRESHOLD) &&
        (abs(sdValB - DELIMAN_B_STDEV) < THRESHOLD) &&
        (abs(sdValG - DELIMAN_G_STDEV) < THRESHOLD) &&
        (abs(sdValR - DELIMAN_R_STDEV) < THRESHOLD))
    {
        pub("deliman");
        ROS_INFO("published deliman");
        dispName = "DELI MAN";
        putText(frame, dispName, posText1, FONT_HERSHEY_COMPLEX, 1.2, WHITE, 2, 8); // displaying recognised names
    }  
    else
    {
        pub("unknown");
        ROS_INFO("published unknown");
        dispName = "UNKNOWN";
        putText(frame, dispName, posText1, FONT_HERSHEY_COMPLEX, 1.2, WHITE, 2, 8); // displaying
    }  
    
    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, frame);
    cv::waitKey(3);
    
    // Output modified video stream
    //image_pub_.publish(cv_ptr->toImageMsg());
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

