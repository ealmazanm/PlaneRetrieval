#pragma once
#include "XnCppWrapper.h"
#include "CameraProperties.h"
#include <boost/thread.hpp>
#include <boost/date_time.hpp> 
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "DynamicPlane.h"
#include "stdio.h"
#include "Utils.h"
#include "filePaths.h"
#include "Plane.h"
#include <Windows.h>
#include <iostream>
#include <fstream>
#include <string>
#include <new>
#include <list>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/fstream.hpp>
#include <cvblob.h>


//using namespace boost::filesystem; 
using namespace std;
using namespace xn;
using namespace cvb;
using std::string;


const char* SEED = "Seed";
const char* DEPTH = "Depth";
const char* RGB =  "RGB";
const int MAX_DEPTH = 10000;
//const double COLORFILTER_THRESHOLD = 0.01;
const int AREA_THRESHOLD = 600;
//class PlaneRetrieval
//{
//}