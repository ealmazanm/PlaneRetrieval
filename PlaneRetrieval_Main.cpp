#include "PlaneRetrieval.h"

ofstream outDebug(filePaths::DEBUG_FILEPATH, ios::out);

void selectROI_callBack(int event, int x, int y, int flags, void* param)
{
	Plane* plane = (Plane*)param;
	if (event == CV_EVENT_LBUTTONDOWN) 
	{
		XnPoint3D p;
		p.X = x;
		p.Y = y;
		if (plane->getInitPoint().X == -1)
			plane->setInitPoint(p);
		else
			plane->setEndPoint(p);
	}
}



int getNumberOfFiles(char* folderPath)
{
	int out = 0;
	boost::filesystem::directory_iterator end ;
	//depthMap cam1
	for( boost::filesystem::directory_iterator iter(folderPath) ; iter != end ; ++iter )
      if ( !is_directory( *iter ) )
		  out++;

	return out;
}

int getNumberOfPlanes()
{
	int out, tmp;
	out = 0;

	boost::filesystem::directory_iterator end;
	for( boost::filesystem::directory_iterator iter(filePaths::CAM1_CALIBRATION_DATA) ; iter != end ; ++iter )
	{
      if (is_directory( *iter ))
	  {
		  string strFile = iter->path().string();
		  strFile.replace(strFile.find('\\'),1, "/");
		  char *charFile=new char[strFile.size()+1];
		  charFile[strFile.size()]=0;
		  memcpy(charFile,strFile.c_str(),strFile.size());
		  string folderName = iter->path().filename().string();
		  if (!folderName.compare(SEED) || !folderName.compare(RGB) || !folderName.compare(DEPTH))
		  {
			  tmp = getNumberOfFiles(charFile);
			  if (out == 0)
				  out = tmp;
			  else if (out != tmp)
			  {
				  cout << "Error in number of calibration planes" << endl;
				  exit(1);
			  }
		  }
	  }
	}
	
	for( boost::filesystem::directory_iterator iter(filePaths::CAM2_CALIBRATION_DATA) ; iter != end ; ++iter )
	{
      if (is_directory( *iter ))
	  {
		  string strFile = iter->path().string();
		  strFile.replace(strFile.find('\\'),1, "/");
		  char *charFile=new char[strFile.size()+1];
		  charFile[strFile.size()]=0;
		  memcpy(charFile,strFile.c_str(),strFile.size());
		  		  
		  string folderName = iter->path().filename().string();
		  if (!folderName.compare(SEED) || !folderName.compare(RGB) || !folderName.compare(DEPTH))
		  {
			  tmp = getNumberOfFiles(charFile);
			  if (out != tmp)
			  {
				  cout << "Error in number of calibration planes" << endl;
				  exit(1);
			  }
		  }
	  }
	}
	return out;
}


void colorFilter_HSV(list<XnPoint3D>* points, const XnRGB24Pixel* rgbMap, const XnDepthPixel *depthMap, const vector<vector<double>>* hist, IplImage* hsvImg)
{

	//IplImage *rgbImg = cvCreateImage(cvSize(640,480), 8, 3);
	//IplImage * hsvImg = cvCreateImage(cvGetSize(rgbImg), rgbImg->depth, 3);
	//Utils::fillImageData(rgbImg, rgbMap, depthMap);
	//cvCvtColor(rgbImg, hsvImg, CV_BGR2HSV);
	//IplImage* binaryImg = cvCreateImage(cvSize(640, 480), 8, 1);
	//Utils::initImage(binaryImg, 0);

	int step = 256/8;
	for (int y = 0; y < XN_VGA_Y_RES; y++)
	{
		const uchar* ptr = (const uchar*)(hsvImg->imageData + y*hsvImg->widthStep);

		//uchar* binaryPtr = (uchar*)(binaryImg->imageData + y*binaryImg->widthStep);
		for (int x = 0; x < XN_VGA_X_RES; x++)
		{
				int h = *ptr++;
				int s = *ptr++;
				int v = *ptr++;
				int hBin = h/step;
				int sBin = s/step;
				double prob = (*hist)[hBin][sBin];
				if (prob > Utils::COLORFILTER_THRESHOLD)
				{
					XnPoint3D p;
					p.X = x;
					p.Y = y;
					p.Z = depthMap[y * XN_VGA_X_RES + x];
					points->push_back(p);

					//binaryPtr[x] = 255;
				}
		}
	}

	//cvSaveImage(fileName, binaryImg);
}


/*
Create a labelled image with connected components. The component with biggest area is the point list result
*/
void labellingNoiseFilter(list<XnPoint3D>* pointsSrc, Plane* plane, const XnDepthPixel* depthMap)
{
	//initializes the mask and binary image
//	IplImage* dst = cvCreateImage(cvSize(XN_VGA_X_RES, XN_VGA_Y_RES), IPL_DEPTH_8U, 3);
	IplImage* img = cvCreateImage(cvSize(XN_VGA_X_RES, XN_VGA_Y_RES), IPL_DEPTH_8U, 1);
//	IplImage* tmp = cvCreateImage(cvSize(XN_VGA_X_RES, XN_VGA_Y_RES), IPL_DEPTH_8U, 1);
	IplImage* labelImg = cvCreateImage(cvGetSize(img), IPL_DEPTH_LABEL, 1);
	CvBlobs blobs;
	
	Utils::initImage(img , 0);
	//updates the binaryImage with the points in 'points2Check'
	list<XnPoint3D>::iterator it;
	for (it=pointsSrc->begin(); it!=pointsSrc->end(); ++it)
	{
		XnPoint3D p = *it;
		((uchar*)(img->imageData + (int)p.Y*img->widthStep))[(int)p.X] = 255;
	}
	
	cvThreshold(img, img, 100,255, CV_THRESH_BINARY);
	
	unsigned int result = cvLabel(img, labelImg, blobs);	

	cvFilterByLabel(blobs,cvGreaterBlob(blobs));

//	cvRenderBlobs(labelImg, blobs, img, dst);

//	cvSaveImage(fileName, dst);

	unsigned int area =  blobs.begin()->second->area;
    
	if (area > AREA_THRESHOLD)
	{
		XnPoint3D initPoint, endPoint; 

		int minX = blobs.begin()->second->minx;
		int maxX = blobs.begin()->second->maxx;
		int minY = blobs.begin()->second->miny;
		int maxY = blobs.begin()->second->maxy;
		int offX = abs(maxX-minX)*20/100;
		int offY = abs(maxY-minY)*20/100;
			
		initPoint.X = minX + offX;
		initPoint.Y = minY + offY;
		endPoint.X = maxX - offX;
		endPoint.Y = maxY - offY;

		plane->setInitPoint(initPoint);
		plane->setEndPoint(endPoint);
	}
	cvReleaseImage(&img);
	cvReleaseImage(&labelImg);
}

/*
Generates a list of points with all the points of the ROI defined by 'plane'
*/
void generateListPoint(Plane* plane, list<XnPoint3D>* lst, const XnDepthPixel* pDepthMap, int maskSize)
{
	int maskOffset = (maskSize/2);

	int totalDepth = 0;
	double avgDepth;

	int contX = plane->getInitPoint().X;
	int contY;
	while (contX < plane->getEndPoint().X)
	{
		contY = plane->getInitPoint().Y;
		while (contY < plane->getEndPoint().Y)
		{
			XnPoint3D p;
			p.X = contX;
			p.Y = contY;
			p.Z = pDepthMap[contY*XN_VGA_X_RES+contX];
			totalDepth += p.Z;
			lst->push_back(p);
			contY += (2*maskOffset); 
		}
		contX += (2*maskOffset);
	}
	//Remove noise
	avgDepth = totalDepth/lst->size();
	list<XnPoint3D>::iterator it = lst->begin();
	while(it != lst->end())
	{
		XnPoint3D p = *it;
		if (abs(p.Z-avgDepth) > 450) //to control the seed does not have any outlier.
			it = lst->erase(it);
		else
			it++;
	}
}

/*
Allows the user to select a ROI in the image. (It will be used as a seed for the growing plane)
*/
void getROISeed(char* windowName, Plane* plane, IplImage* depthImage)
{
	cvSetMouseCallback(windowName, selectROI_callBack, (Plane*)plane);
	cvShowImage(windowName, depthImage);
	while (!plane->isROISelected())
		cvWaitKey(1);

	cvRectangle(depthImage, cvPoint(plane->getInitPoint().X, plane->getInitPoint().Y), cvPoint(plane->getEndPoint().X, plane->getEndPoint().Y),cvScalar(0,0,255));
	cvShowImage(windowName, depthImage);
	cvWaitKey(1);
//	cvDestroyWindow(windowName);
}

/*
Calculates the normal vector of a plane defined by 'param' (Ax+By+C=Z) adding the restriction
of being a unit normal vector.
*/
void unitNormal(CvMat* normal, const CvMat* param)
{
	CvMat* normalVect = cvCreateMat(3,1,CV_32FC1);
	CV_MAT_ELEM( *normalVect, float, 0, 0) = CV_MAT_ELEM( *param, float, 0, 0);
	CV_MAT_ELEM( *normalVect, float, 1, 0) = CV_MAT_ELEM( *param, float, 1, 0);
	CV_MAT_ELEM( *normalVect, float, 2, 0) = -1;

	float lengthNormal = cvNorm(normalVect);
	CV_MAT_ELEM( *normal, float, 0, 0) = CV_MAT_ELEM( *normalVect, float, 0, 0)/lengthNormal;
	CV_MAT_ELEM( *normal, float, 1, 0) = CV_MAT_ELEM( *normalVect, float, 1, 0)/lengthNormal;
	CV_MAT_ELEM( *normal, float, 2, 0) = CV_MAT_ELEM( *normalVect, float, 2, 0)/lengthNormal;
}



void getStorePlanes_FromSeeds(int maskSize, IplImage* depthImg, char* nameWind_D, CameraProperties* cam, char* strIdPlane, char* fileNames1[6])
{
	ofstream centrStream, normalStream;
	ifstream depthStream, seedStream;

	Utils::createGeneralOutStream(&normalStream, "Normals_", atoi(strIdPlane), cam->getCamId());
	
	Plane plane;
	list<XnPoint3D> planePointLst; // seed
	int rgbPlanes[] = {0,0,255};  //fill color

	int total = XN_VGA_X_RES*XN_VGA_Y_RES;	
	unsigned short depth[MAX_DEPTH];
	char *depth_data;

	char depthTmp[130];
	char normalTmp[130];
	char paramTmp[130];
	char centrTmp[130];
	char seedTmp[130];

	//Create the name of the file
	strcpy(depthTmp, fileNames1[0]);
	strcpy(normalTmp, fileNames1[2]);
	strcpy(paramTmp, fileNames1[3]);
	strcpy(centrTmp, fileNames1[4]);
	strcpy(seedTmp, fileNames1[5]);

	strcat(depthTmp,strIdPlane);
	strcat(normalTmp,strIdPlane);
	strcat(paramTmp, strIdPlane);
	strcat(centrTmp,strIdPlane);
	strcat(seedTmp, strIdPlane);

	strcat(depthTmp, ".txt");
	strcat(normalTmp, ".xml");
	strcat(paramTmp, ".xml");
	strcat(centrTmp, ".txt");
	strcat(seedTmp, ".txt");

	//Read depthMap and rgbMap from file
	XnDepthPixel* depthMap = new XnDepthPixel[total];
	depthStream.open(depthTmp);
	for (int i = 0; i < total; i++)
	{
		int depth;
		depthStream >> depth;
		depthMap[i] = depth;
	}
	depthStream.close();

	//create depth image
	depth_data = (char*) malloc(640*480*3);
	Utils::raw2depth(depth, MAX_DEPTH);
	Utils::depth2rgb(depthMap, depth, depth_data);
	cvSetData(depthImg, depth_data, 640*3);

	seedStream.open(seedTmp);
	XnPoint3D initPoint, endPoint;
	seedStream >> initPoint.X; seedStream >> initPoint.Y; seedStream >> initPoint.Z;
	seedStream >> endPoint.X; seedStream >> endPoint.Y; seedStream >> endPoint.Z;
	seedStream.close();
	plane.setInitPoint(initPoint);
	plane.setEndPoint(endPoint);

	generateListPoint(&plane, &planePointLst, depthMap, maskSize);

	if (planePointLst.size() == 0)
	{
		ofstream normalStreamII(normalTmp);
		ofstream paramStream(paramTmp);
		centrStream.open(centrTmp);
		normalStreamII.close();
		paramStream.close();
		centrStream.close();
		cout << "Error. Remove plane number " << strIdPlane << endl;
	}
	else
	{
		cvNamedWindow(nameWind_D, 1);
		int idPlane = atoi(strIdPlane);
		DynamicPlane planeCreator(&planePointLst, depthImg->width, depthImg->height, maskSize, rgbPlanes, idPlane, cam);
		planeCreator.makePlaneGrow(nameWind_D, depthImg, depthMap);
		plane.setParameters(planeCreator.getPlaneParameters());
		CvMat* unitNorm = cvCreateMat(3,1, CV_32FC1);
		unitNormal(unitNorm, plane.getParameters());
		plane.setNormal(unitNorm);
		cvSave(normalTmp, plane.getNormal());
		cvSave(paramTmp, plane.getParameters());

		XnPoint3D* centroid2D = planeCreator.getCentroid();
		XnPoint3D centroid3D;
		cam->backProjectPoint(centroid2D, &centroid3D);

		//write centroid
		centrStream.open(centrTmp);
		centrStream << centroid3D.X << " " << centroid3D.Y << " " << centroid3D.Z;
		centrStream.close();

		cvCircle(depthImg, cvPoint(centroid2D->X, centroid2D->Y),2, cvScalar(0,0,0), 3);
		cvShowImage(nameWind_D, depthImg);
		cvWaitKey(500);


		//DEBUG BEGIN
		normalStream << CV_MAT_ELEM( *plane.getNormal(), float, 0, 0) << endl;
		normalStream << CV_MAT_ELEM( *plane.getNormal(), float, 1, 0) << endl;
		normalStream << CV_MAT_ELEM( *plane.getNormal(), float, 2, 0) << endl;
		//DEBUG END	


		cout << "Plane " << idPlane << " captured in camera " << cam->getCamId() << endl;

		cvDestroyWindow(nameWind_D);
	}
	free(depth_data);

}

void getStorePlanes(int maskSize, IplImage* depthImg, char* nameWind_D, CameraProperties* cam, const vector<vector<double>>* hist, char* strIdPlane, char* fileNames[6])
{
	ofstream centrStream, normalStream, seedStream;
	ifstream depthStream, rgbStream;

	Utils::createGeneralOutStream(&normalStream, "Normals_", atoi(strIdPlane), cam->getCamId());
	
	Plane plane;
	list<XnPoint3D> planePointLst; // seed
	int rgbPlanes[] = {0,0,255};  //fill color

	int total = XN_VGA_X_RES*XN_VGA_Y_RES;	
	unsigned short depth[MAX_DEPTH];
	char *depth_data;

	char depthTmp[130];
	char rgbTmp[130];
	char normalTmp[130];
	char paramTmp[130];
	char centrTmp[130];
	char seedTmp[130];

	//Create the name of the file
	strcpy(depthTmp, fileNames[0]);
	strcpy(rgbTmp, fileNames[1]);
	strcpy(normalTmp, fileNames[2]);
	strcpy(paramTmp, fileNames[3]);
	strcpy(centrTmp, fileNames[4]);
	strcpy(seedTmp, fileNames[5]);

	strcat(depthTmp,strIdPlane);
	strcat(rgbTmp, strIdPlane);
	strcat(normalTmp,strIdPlane);
	strcat(paramTmp, strIdPlane);
	strcat(centrTmp,strIdPlane);
	strcat(seedTmp, strIdPlane);

	strcat(depthTmp, ".txt");
	strcat(rgbTmp, ".txt");
	strcat(normalTmp, ".xml");
	strcat(paramTmp, ".xml");
	strcat(centrTmp, ".txt");
	strcat(seedTmp, ".txt");

	//Read depthMap and rgbMap from file
	XnDepthPixel* depthMap = new XnDepthPixel[total];
	XnRGB24Pixel* rgbMap = new XnRGB24Pixel[total];
	depthStream.open(depthTmp);
	rgbStream.open(rgbTmp);
	for (int i = 0; i < total; i++)
	{
		int depth, r,g,b;
		depthStream >> depth;
		depthMap[i] = depth;
		rgbStream >> r; rgbStream >> g; rgbStream >> b;
		rgbMap[i].nRed = r; rgbMap[i].nGreen = g; rgbMap[i].nBlue = b;
	}
	depthStream.close();
	rgbStream.close();

	//create depth image
	depth_data = (char*) malloc(640*480*3);
	Utils::raw2depth(depth, MAX_DEPTH);
	Utils::depth2rgb(depthMap, depth, depth_data);
	cvSetData(depthImg, depth_data, 640*3);

	IplImage *rgbImg = cvCreateImage(cvSize(640,480), 8, 3);
	IplImage * hsvImg = cvCreateImage(cvGetSize(rgbImg), rgbImg->depth, 3);
	Utils::fillImageData(rgbImg, rgbMap, depthMap);
	cvCvtColor(rgbImg, hsvImg, CV_BGR2HSV);
	colorFilter_HSV(&planePointLst, rgbMap, depthMap, hist, hsvImg); //hsv filter

	labellingNoiseFilter(&planePointLst, &plane, depthMap);
	planePointLst.clear();
	cvNamedWindow(nameWind_D, 1);
	if (!plane.isROISelected())
	{
		cout << "Not found proper seed for plane " << strIdPlane << " (camera " << cam->getCamId() << "). Select it manually." << endl;
		getROISeed(nameWind_D, &plane, depthImg);
	}

	//write seed in file
	seedStream.open(seedTmp);
	seedStream << plane.getInitPoint().X << " " << plane.getInitPoint().Y << " " << plane.getInitPoint().Z << endl;
	seedStream << plane.getEndPoint().X << " " << plane.getEndPoint().Y << " " << plane.getEndPoint().Z << endl;
	seedStream.close();

	generateListPoint(&plane, &planePointLst, depthMap, maskSize);

//	cvNamedWindow(nameWind_D, 1);
	int idPlane = atoi(strIdPlane);
	DynamicPlane planeCreator(&planePointLst, depthImg->width, depthImg->height, maskSize, rgbPlanes, idPlane, cam);
	planeCreator.makePlaneGrow(nameWind_D, depthImg, depthMap);
	plane.setParameters(planeCreator.getPlaneParameters());
	CvMat* unitNorm = cvCreateMat(3,1, CV_32FC1);
	unitNormal(unitNorm, plane.getParameters());
	plane.setNormal(unitNorm);
	cvSave(normalTmp, plane.getNormal());
	cvSave(paramTmp, plane.getParameters());

	XnPoint3D* centroid2D = planeCreator.getCentroid();
	XnPoint3D centroid3D;
	cam->backProjectPoint(centroid2D, &centroid3D);

	//write centroid
	centrStream.open(centrTmp);
	centrStream << centroid3D.X << " " << centroid3D.Y << " " << centroid3D.Z;
	centrStream.close();

	cvCircle(depthImg, cvPoint(centroid2D->X, centroid2D->Y),2, cvScalar(0,0,0));
	cvShowImage(nameWind_D, depthImg);
	cvWaitKey(2000);


//DEBUG BEGIN
normalStream << CV_MAT_ELEM( *plane.getNormal(), float, 0, 0) << endl;
normalStream << CV_MAT_ELEM( *plane.getNormal(), float, 1, 0) << endl;
normalStream << CV_MAT_ELEM( *plane.getNormal(), float, 2, 0) << endl;
//DEBUG END	

	
	cout << "Plane " << idPlane << " captured in camera " << cam->getCamId() << endl;

	cvDestroyWindow(nameWind_D);
	free(depth_data);

}

void getPlanes(int maskSize, IplImage* depthImg, char* nameWind_D, CameraProperties* cam, const vector<vector<double>>* hist, char* strIdPlane, char* fileNames1[6])
{
	ofstream depthStream, rgbStream, centrStream, seedStream;
	Plane plane;

	ofstream planeNormal;
	Utils::createGeneralOutStream(&planeNormal, "Normals_", atoi(strIdPlane), cam->getCamId());

	list<XnPoint3D> planePointLst; // seed
	int rgbPlanes[] = {0,0,255};  //fill color

	int total = XN_VGA_X_RES*XN_VGA_Y_RES;	
	unsigned short depth[MAX_DEPTH];
	char *depth_data;

	char depthTmp[130];
	char rgbTmp[130];
	char normalTmp[130];
	char paramTmp[130];
	char centrTmp[130];
	char seedTmp[130];

	cam->getContext()->WaitAndUpdateAll();	
	const XnDepthPixel* dM = (XnDepthPixel*) cam->getDepthNode()->GetDepthMap();		
	const XnRGB24Pixel* rgbM =  (XnRGB24Pixel*) cam->getImageNode()->GetRGB24ImageMap();	

	/*char rgbImgTmp[130];
	strcpy(rgbImgTmp, "D:/CameraCalibrations/dataCalib/cam_");
	char strIdCam[10];
	itoa(cam->getCamId(), strIdCam, 10);
	strcat(rgbImgTmp, strIdCam);
	strcat(rgbImgTmp, strIdPlane);
	strcat(rgbImgTmp, ".jpg");
	IplImage* rgbImg = cvCreateImage(cvSize(XN_VGA_X_RES, XN_VGA_Y_RES), IPL_DEPTH_8U, 3);
	Utils::fillImageDataFull(rgbImg, rgbM);
	cvSaveImage(rgbImgTmp, rgbImg);*/

	//Create the name of the file
	strcpy(depthTmp, fileNames1[0]);
	strcpy(rgbTmp, fileNames1[1]);
	strcpy(normalTmp, fileNames1[2]);
	strcpy(paramTmp, fileNames1[3]);
	strcpy(centrTmp, fileNames1[4]);
	strcpy(seedTmp, fileNames1[5]);

	

	strcat(depthTmp,strIdPlane);
	strcat(rgbTmp, strIdPlane);
	strcat(normalTmp,strIdPlane);
	strcat(paramTmp, strIdPlane);
	strcat(centrTmp,strIdPlane);
	strcat(seedTmp, strIdPlane);

	strcat(depthTmp, ".txt");
	strcat(rgbTmp, ".txt");
	strcat(normalTmp, ".xml");
	strcat(paramTmp, ".xml");
	strcat(centrTmp, ".txt");
	strcat(seedTmp, ".txt");

	depthStream.open(depthTmp);
	rgbStream.open(rgbTmp);
	centrStream.open(centrTmp);
	seedStream.open(seedTmp);

	//Copy depth and rgb map
	for (int i = 0; i < total; i++)
	{
		depthStream << dM[i] << " ";
		rgbStream << (int)rgbM[i].nRed << " " << (int)rgbM[i].nGreen << " " << (int)rgbM[i].nBlue << " ";
	}
	depthStream.close();
	rgbStream.close();

	//create depth image
	depth_data = new char[640*480*3];
	Utils::raw2depth(depth, MAX_DEPTH);
	Utils::depth2rgb(dM, depth, depth_data);
	cvSetData(depthImg, depth_data, 640*3);

	cvNamedWindow(nameWind_D, 1);

	IplImage *rgbImg = cvCreateImage(cvSize(640,480), 8, 3);
	IplImage * hsvImg = cvCreateImage(cvGetSize(rgbImg), rgbImg->depth, 3);
	Utils::fillImageData(rgbImg, rgbM, dM);
	cvCvtColor(rgbImg, hsvImg, CV_BGR2HSV);
	colorFilter_HSV(&planePointLst, rgbM, dM, hist, hsvImg); //hsv filter

	labellingNoiseFilter(&planePointLst, &plane, dM);
	planePointLst.clear();

	if (!plane.isROISelected())
	{
		cout << "Not found proper seed for plane " << strIdPlane << " (camera " << cam->getCamId() << "). Select it manually." << endl;
		getROISeed(nameWind_D, &plane, depthImg);
	}

	generateListPoint(&plane, &planePointLst, dM, maskSize);

	if (planePointLst.size() == 0)
	{
		cout << "NOt found proper seed for plane " << strIdPlane << " (camera " << cam->getCamId() << "). Select it manually." << endl;
		getROISeed(nameWind_D, &plane, depthImg);
		generateListPoint(&plane, &planePointLst, dM, maskSize);
	}

	//write seed in file
	seedStream << plane.getInitPoint().X << " " << plane.getInitPoint().Y << " " << plane.getInitPoint().Z << endl;
	seedStream << plane.getEndPoint().X << " " << plane.getEndPoint().Y << " " << plane.getEndPoint().Z << endl;
	seedStream.close();

	

	outDebug << "Cam " << cam->getCamId() << " ready to grow" << endl;

	int idPlane = atoi(strIdPlane);
	DynamicPlane planeCreator(&planePointLst, depthImg->width, depthImg->height, maskSize, rgbPlanes, idPlane, cam, hist, hsvImg, &outDebug);
	outDebug << "Cam " << cam->getCamId() << " dynamic plane instanciated" << endl;
	planeCreator.makePlaneGrow(nameWind_D, depthImg, dM);

	outDebug << "Cam " << cam->getCamId() << " growth" << endl;
	plane.setParameters(planeCreator.getPlaneParameters());
	CvMat* unitNorm = cvCreateMat(3,1, CV_32FC1);
	unitNormal(unitNorm, plane.getParameters());
	plane.setNormal(unitNorm);
	cvSave(normalTmp, plane.getNormal());
	cvSave(paramTmp, plane.getParameters());

	XnPoint3D* centroid2D = planeCreator.getCentroid();
	XnPoint3D centroid3D;
	cam->backProjectPoint(centroid2D, &centroid3D);

	//write centroid
	centrStream << centroid3D.X << " " << centroid3D.Y << " " << centroid3D.Z;
	centrStream.close();

	cvCircle(depthImg, cvPoint(centroid2D->X, centroid2D->Y),2, cvScalar(0,0,0),2);
	cvShowImage(nameWind_D, depthImg);
	cvWaitKey(2000);


//DEBUG BEGIN
planeNormal << CV_MAT_ELEM( *plane.getNormal(), float, 0, 0) << endl;
planeNormal << CV_MAT_ELEM( *plane.getNormal(), float, 1, 0) << endl;
planeNormal << CV_MAT_ELEM( *plane.getNormal(), float, 2, 0) << endl;
//DEBUG END	

	
	cout << "Plane " << idPlane << " captured in camera " << cam->getCamId() << endl;

	cvDestroyWindow(nameWind_D);
	delete(depth_data);
	cvReleaseImage(&rgbImg);
	cvReleaseImage(&hsvImg);
	cvReleaseMat(&unitNorm);
}

void resizeHistogram_HSV(vector<vector<double>>* hist)
{
	hist->resize(8);
	for (int i = 0; i < 8; i++)
	{
		(*hist)[i].resize(8);
	}

	//initializes the array
	for (int i = 0; i < 8; i++)
		for(int j = 0; j < 8; j++)
				(*hist)[i][j] = 0.;
}

/*
Load all the training images and creates a 3D (rgb) histogram of probability
*/
void trainColor_HSV(vector<vector<double>>* hist)
{
	IplImage* hsvImg;

	double binSize = 256/8;

	char filePath[80];
	char strId[20];
	int totalPixels = 0;
		
	//Loop through all the files in the training directory
	boost::filesystem::directory_iterator end ;
    for( boost::filesystem::directory_iterator iter(filePaths::TRAINIGN_IMAGES_FILEPATH) ; iter != end ; ++iter )
      if ( !is_directory( *iter ) )
	  {
		  //convert path from string to char (it can be loaded with cvLoadImage)
		  string strFile = iter->path().string();			
		  char *charFile=new char[strFile.size()+1];
		  charFile[strFile.size()]=0;
		  memcpy(charFile,strFile.c_str(),strFile.size());

		  IplImage* rgbImg = cvLoadImage(charFile);
		  hsvImg = cvCreateImage(cvGetSize(rgbImg), rgbImg->depth, 3);
		  /* Convert from Red-Green-Blue to Hue-Saturation-Value */
		  cvCvtColor(rgbImg, hsvImg, CV_BGR2HSV);
		  for (int y = 0; y < hsvImg->height; y++)
		  {
			  const uchar* ptr = (const uchar*)(hsvImg->imageData + y*hsvImg->widthStep);
			  for (int x = 0; x < hsvImg->width; x++)
			  {
				  int h = *ptr++;
				  int s = *ptr++;
				  int v = *ptr++;
				  int hBin = h/binSize;
				  int sBin = s/binSize;
				 // int vBin = v/vBinSize;
				  (*hist)[hBin][sBin] += 1; 
				  totalPixels++;
			  }
		  }
		  delete(charFile);
	  }
	//calculate probabilities
	for (int i = 0; i < 8; i ++)
	{
		for (int j = 0; j < 8; j ++)
		{
			(*hist)[i][j] /= totalPixels;
			
			outDebug <<  "Bin [" <<i<<"]["<<j<<"]=" << (*hist)[i][j] << endl;
		}
	}
}


//Fill the "fileNames" with the absolute path of the files that define the planes
void fillPaths(char** fileNames, int camId)
{
	char strCamId[10];
	itoa(camId, strCamId, 10);
	if (camId == 1)
	{
		strcpy(fileNames[0], filePaths::CAM1_CALIB_DEPTHMAP);
		strcpy(fileNames[1], filePaths::CAM1_CALIB_RGBMAP);
		strcpy(fileNames[2], filePaths::CAM1_CALIB_NORMALS);
		strcpy(fileNames[3], filePaths::CAM1_CALIB_PARAMETERS);
		strcpy(fileNames[4], filePaths::CAM1_CALIB_CENTROIDS);
		strcpy(fileNames[5], filePaths::CAM1_CALIB_SEED);
		
		
	}
	else
	{
		strcpy(fileNames[0], filePaths::CAM2_CALIB_DEPTHMAP);
		strcpy(fileNames[1], filePaths::CAM2_CALIB_RGBMAP);
		strcpy(fileNames[2], filePaths::CAM2_CALIB_NORMALS);
		strcpy(fileNames[3], filePaths::CAM2_CALIB_PARAMETERS);
		strcpy(fileNames[4], filePaths::CAM2_CALIB_CENTROIDS);
		strcpy(fileNames[5], filePaths::CAM2_CALIB_SEED);
	}

	strcat(fileNames[0], "Depth ");
	strcat(fileNames[0], strCamId);

	strcat(fileNames[1], "RGB ");
	strcat(fileNames[1], strCamId);

	strcat(fileNames[2], "Normal ");
	strcat(fileNames[2], strCamId);

	strcat(fileNames[3], "Param ");
	strcat(fileNames[3], strCamId);

	strcat(fileNames[4], "Centroid ");
	strcat(fileNames[4], strCamId);

	strcat(fileNames[5], "Seed ");
	strcat(fileNames[5], strCamId);
}


/*
Uses images already stored to create planes with a maskSize specified by the user.
*/
void createNewPlanes(CameraProperties* cam1, CameraProperties* cam2, const vector<vector<double>>* hsvHst, char** fileNames1, char** fileNames2)
{
	int maskSize;
	cout << "Choose a mask size for the plane fitting process (3, 5, 7, 9,...): ";
	cin >> maskSize;

	int refPlaneNum = getNumberOfFiles(filePaths::CAM1_CALIB_DEPTHMAP);
	
	IplImage* depthImg1 = cvCreateImageHeader(cvSize(640,480), 8, 3);
	IplImage* depthImg2 = cvCreateImageHeader(cvSize(640,480), 8, 3);
	ifstream depthStream1, depthStream2;

	//Create window name
	char nameWind1_D[50];
	char nameWind2_D[50];
	strcpy(nameWind1_D, "Depth 1");
	strcpy(nameWind2_D, "Depth 2");
	char idPlane[5];

	int option;
	cout << "1. Used loaded seeds." << endl;
	cout << "2. Generate new seeds." << endl;
	cin >> option;

	if (option == 1)
	{
//		cvNamedWindow(nameWind1_D);
//		cvNamedWindow(nameWind2_D);
		for (int i = 0; i < refPlaneNum; i++)
		{
			itoa(i, idPlane, 10);
			boost::thread thr(getStorePlanes_FromSeeds, maskSize, depthImg2, nameWind2_D, cam2, idPlane, fileNames2);
			getStorePlanes_FromSeeds(maskSize, depthImg1, nameWind1_D, cam1, idPlane, fileNames1);
			thr.join();
		}
//		cvDestroyAllWindows();
	}
	else
	{
		for (int i = 0; i < refPlaneNum; i++)
		{
			itoa(i, idPlane, 10);
			boost::thread thr(getStorePlanes, maskSize, depthImg2, nameWind2_D, cam2, hsvHst, idPlane, fileNames2);
			getStorePlanes(maskSize, depthImg1, nameWind1_D, cam1, hsvHst, idPlane, fileNames1);
			thr.join();
		}
	}
}


/*
Capture new images (depth and rgb) in both cameras, perform a color filter and a plane fitting. Generate new planes
*/
void captureNewPlanes(CameraProperties* cam1, CameraProperties* cam2, const vector<vector<double>>* hsvHst, char** fileNames1, char** fileNames2)
{
	int nPlanes;
	cout << "How many plane correspondences will be added to the plane set? ";
	cin >> nPlanes;
	int maskSize = 9;
	cout << "What mask size? (3, 5, 7, 9,...) ";
	cin >> maskSize;

	//check the number of planes stored already (also check if this number is consistent in all the calibration data folders)
	int refPlaneNum = getNumberOfPlanes();
	int total = refPlaneNum + nPlanes;

	//Loop to retrieve the new planes (depthMap, rgbMap, seedPoints, centroid, normal and parameters)
	IplImage* depthImg1 = cvCreateImageHeader(cvSize(640,480), 8, 3);
	IplImage* depthImg2 = cvCreateImageHeader(cvSize(640,480), 8, 3);

	//Create window name
	char nameWind1_D[50];
	char nameWind2_D[50];
	strcpy(nameWind1_D, "Depth 1");
	strcpy(nameWind2_D, "Depth 2");
	char idPlane[5];

	cam1->getContext()->StartGeneratingAll();
	cam2->getContext()->StartGeneratingAll();
	for (int i = refPlaneNum; i < total; i++)
	{
		itoa(i, idPlane, 10);
		boost::thread thr(getPlanes, maskSize, depthImg2, nameWind2_D, cam2, hsvHst, idPlane, fileNames2);
		getPlanes(maskSize, depthImg1, nameWind1_D, cam1, hsvHst, idPlane, fileNames1);
		thr.join();
		Beep(500, 650);
		Sleep(2000);

	}
	cam1->getContext()->StopGeneratingAll();
	cam2->getContext()->StopGeneratingAll();

	cvReleaseImageHeader(&depthImg1);
	cvReleaseImageHeader(&depthImg2);
}


int main()
{
	
	CameraProperties cam1, cam2;
	Utils::rgbdInitAligned(&cam1, &cam2);
	Utils::initIntrinsicParameters(&cam1);
	Utils::initIntrinsicParameters(&cam2);

	vector<vector<double>> hsvHst;
	resizeHistogram_HSV(&hsvHst);
	trainColor_HSV(&hsvHst);

	//create root paths
	char depthPlane1_fn[130], depthPlane2_fn[130];
	char rgbPlane1_fn[130], rgbPlane2_fn[130];
	char normals1_fn[130], normals2_fn[130];
	char parameters1_fn[130], parameters2_fn[130];
	char centroids1_fn[130], centroids2_fn[130];
	char seeds1_fn[130], seeds2_fn[130];
		
	char* fileNames1[6];
	char* fileNames2[6];

	fileNames1[0] = depthPlane1_fn; fileNames2[0] = depthPlane2_fn;
	fileNames1[1] = rgbPlane1_fn;	fileNames2[1] = rgbPlane2_fn;
	fileNames1[2] = normals1_fn;	fileNames2[2] = normals2_fn;
	fileNames1[3] = parameters1_fn; fileNames2[3] = parameters2_fn;
	fileNames1[4] = centroids1_fn;	fileNames2[4] = centroids2_fn;
	fileNames1[5] = seeds1_fn;		fileNames2[5] = seeds2_fn;

	fillPaths(fileNames1, 1);
	fillPaths(fileNames2, 2);

	int option;
	cout << "1. Capture new planes." << endl;
	cout << "2. Load planes with different maskSize." << endl;
	cin >> option;

	if (option == 1)
		captureNewPlanes(&cam1, &cam2, &hsvHst, fileNames1, fileNames2);
	else
		createNewPlanes(&cam1, &cam2, &hsvHst, fileNames1, fileNames2);


}

