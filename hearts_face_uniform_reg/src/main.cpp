//#include "main.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include "macros.hpp"

#include <iostream>
#include <fstream>
#include <sstream>
#include <unistd.h>

using namespace std;
using namespace cv;

// TODO: this needs to be rosified - should listen on ip camera topic

/** Global variables */
String face_cascade_name = "haarcascade_frontalface_alt2.xml";
CascadeClassifier face_cascade;
String window_name = "Face Registration";

/** @function main */
int main(void)
{
	string imgType = ".jpg";
	// todo - accept as a ros param
	string root = "/home/turtlebot/tb_ws/src/brl-hearts/hearts_face_uniform/data";
	vector<Mat> images;
	
	cout << "Connecting to webcam, please wait..." << endl;
	VideoCapture capture(0); // open the default camera
	if (!capture.isOpened())  // check if we succeeded
		return -1;

	Mat frame;

	//-- 1. Load the cascades
	if (!face_cascade.load(face_cascade_name)){ printf("--(!)Error loading face cascade\n"); return -1; };

	capture.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
	capture.set(CV_CAP_PROP_FRAME_HEIGHT, 720);
	capture.set(CV_CAP_PROP_FPS, 20);
	//capture.set(CV_CAP_PROP_EXPOSURE, 15);

	cout << "Webcam connected! Data capture will start now." << endl;

	int count = 0;
	while (capture.read(frame))
	{
		if (frame.empty())
		{
			printf(" --(!) No captured frame -- Break!");
			break;
		}

		std::vector<Rect> faces;
		Mat frame_gray;

		cvtColor(frame, frame_gray, COLOR_BGR2GRAY);
		equalizeHist(frame_gray, frame_gray);

		//-- Detect faces
		face_cascade.detectMultiScale(frame_gray, faces, 1.2, 4, 0 | CASCADE_SCALE_IMAGE, Size(MIN_FACE_SIZE, MIN_FACE_SIZE));

		if ((faces.size() > 3) || (faces.size() < 1))
		{
			imshow(window_name, frame);
			int c = waitKey(3);
			if ((char)c == 27) { break; } // escape
			continue;
		}

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

		Rect largestFaceBox = faces[largestIndex];

		Mat oneFace = frame_gray(faces[largestIndex]);

		rectangle(frame, largestFaceBox, GREEN);
		Point posText(largestFaceBox.x, max(1, largestFaceBox.y - 10));

        stringstream ss1;
        ss1 << (CAPTURE_NUM - count);
        string remain = ss1.str();
      
		putText(frame, "Capturing images - " + remain + " remaining...", posText, FONT_HERSHEY_COMPLEX, 1.2, GREEN, 2, 8); // displaying recognised names

		count++;
		if (count > CAPTURE_NUM)
		{
			usleep(2000);
			break;
		}

        stringstream ss2;
        ss2 << count;
        string count_str = ss2.str();
        
		imwrite(root + count_str + imgType, oneFace);
		usleep(200);

		imshow(window_name, frame);
		int c = waitKey(3);
		if ((char)c == 27) { break; } // escape

	}

	return 0;
}

