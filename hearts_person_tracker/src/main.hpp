#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <new>
#include <iostream>
#include <ctype.h>
#include <fstream>
#include <sstream>

#include <thread>  
#include <chrono>
#include <inttypes.h>
#include <Windows.h>
#include <string>

#include "opencv2/opencv.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/contrib_world.hpp"
#include <opencv2/tracking.hpp>
#include <opencv2/face.hpp>

#include "opencv2/core/cuda.hpp"
#include "opencv2/cudaimgproc.hpp"
#include "opencv2/cudafilters.hpp"
#include "opencv2/cudaobjdetect.hpp"
#include "opencv2/cudaarithm.hpp"
#include "opencv2/cudawarping.hpp"

#include "ptg.hpp"
#include "imgCapture.h"
#include "tick_meter.hpp"
#include "PS.hpp"
#include "macros.hpp"
#include "PS.hpp"
#include "class.hpp"
#include "auxiliaryFuncs.hpp"

using namespace FlyCapture2;

