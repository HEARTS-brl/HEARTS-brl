//#include "main.hpp"
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
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include "macros.hpp"
#include <unistd.h>

using namespace std;
using namespace cv;

/** Global variables */
static const std::string OPENCV_WINDOW = "Image window";

// todo - accept as a ros param
static const std::string PATH = "/home/turtlebot/tb_ws/src/brl-hearts/hearts_face_uniform/";

std::string face_cascade_name = PATH + "src/haarcascade_frontalface_alt2.xml";
std::string DATA_PATH = PATH + "data/";

static const String window_name = "Face Registration";
static const string imgType = ".jpg";
    	
class ImageCapturer
{    
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    bool done;
    vector<Mat> frames;
    CascadeClassifier face_cascade;
    
public:
    
    ImageCapturer()
        : it_(nh_)
    {
        done = false;
        
        // Load the cascades
        if (!face_cascade.load(face_cascade_name))
        { 
            printf("--(!)Error loading face cascade\n"); 
            return; // todo; should throw!
        }
        
        // Subscrive to input video feed and publish output video feed
        //image_sub_ = it_.subscribe("/devices/front_door/image", 1, 
        //  &ImageConverter::imageCb, this);
        image_sub_ = it_.subscribe("/roah_ipcam/image", 1, &ImageCapturer::imageCb, this);

        cv::namedWindow(OPENCV_WINDOW);
    }

    ~ImageCapturer()
    {
        destroyWindow(OPENCV_WINDOW);
    }
    
    bool is_done()
    {
        return done;
    } 
    
    vector<Mat> get_frames()
    {
        return frames;
    }
    
private:
    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        if (done)
            return;
            
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
        cout << "rows_"<<frame.rows << " and cols_"<<frame.cols << endl;
        
        if (frame.empty())
		{
			printf(" --(!) No captured frame -- Break!");
			return;
		}
        
		Mat frame_gray;

		cvtColor(frame, frame_gray, COLOR_BGR2GRAY);
		//equalizeHist(frame_gray, frame_gray);
		
		cout << "## 1 ##";
		
		std::vector<Rect> faces;

        //-- Detect faces
        face_cascade.detectMultiScale(frame_gray, faces, 1.2, 4, 0 | CASCADE_SCALE_IMAGE, Size(MIN_FACE_SIZE, MIN_FACE_SIZE));
		
		cout << "## 2 ## - faces: " << faces.size();
		
        if ((faces.size() > 3) || (faces.size() < 1))
        {
	        imshow(window_name, frame_gray);
	        return;
        }
        
        cout << "## 3 ##";
        
        int largestIndex = 0;
        int faceSize = faces[0].width;

        for (size_t i = 0; i < faces.size(); i++)
        {
        	Mat faceROI = frame_gray(faces[i]);
        	if (faces[i].width > faceSize)
        	{
        		largestIndex = i;
        	}
        }

        cout << "## 4 ##";
        
		Mat oneFace = frame_gray(faces[largestIndex]);
		
		stringstream ss1;
        ss1 << (CAPTURE_NUM - frames.size());
        string remain = ss1.str();
      
		cout << "Capturing images - " << remain << " remaining...";
		
		imshow(window_name, frame);
		
		if (frames.size() < CAPTURE_NUM)
		    frames.push_back(oneFace);
		else
		{
		    image_sub_.shutdown();   
	        done = true;
        }
    }
};

/** @function main */
int main(int argc, char** argv)
{
    printf("running!");
    ros::init(argc, argv, "image_capturer");
    ImageCapturer ic;
    ros::Rate r(10);
    while (ros::ok() && !ic.is_done())
    {
        ros::spinOnce();
        r.sleep();
    }
    
    vector<Mat> frames = ic.get_frames();
    
    for (int i = 0; i < frames.size(); i++)
    {
        Mat frame = frames[i];
		
        stringstream ss2;
        ss2 << (i + 1);
        string count_str = ss2.str();
        
		imwrite(DATA_PATH + count_str + imgType, frame);
		usleep(200);        
    }

    //ros::spin();
    return 0;
}

