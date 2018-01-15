#include "2d_depth_detector.h"

using namespace cv;
using namespace std;

DepthDetector::DepthDetector() {
	//positions = new vector<Point>;
}

vector<Point2d> DepthDetector::getPositions() {
  //TO DO: use main camera class
  //logging method
  o3d3xx::Logging::Init();
  //initialise camera constructor expects IP address, using default one
  o3d3xx::Camera::Ptr cam = make_shared<o3d3xx::Camera>("192.168.1.69");
  //create buffer to fetch image
  o3d3xx::ImageBuffer::Ptr img = make_shared<o3d3xx::ImageBuffer>();
  //framegrabber
  o3d3xx::FrameGrabber::Ptr fg =
    make_shared<o3d3xx::FrameGrabber>(
      cam, o3d3xx::IMG_AMP|o3d3xx::IMG_RDIS|o3d3xx::IMG_CART);
  //get frame from camera
  if (! fg->WaitForFrame(img.get(), 2000))
    {
      //TO DO: error handling
      cerr << "Timeout waiting for camera!" << endl;
      //return positions;	
    }
  //2D images
  double min, max;
  ////create depth image
  Mat depthImageRaw = Mat(img->DepthImage()); 
  //Mat xyzImageRaw = Mat(img->XYZImage());
  //resize image for better visibility when debugging
  resize(depthImageRaw,depthImageRaw, Size(depthImageRaw.cols*2,depthImageRaw.rows*2), 2, 2, INTER_CUBIC);
  
  //resize(xyzImageRaw,xyzImageRaw, Size(xyzImageRaw.cols*2,xyzImageRaw.rows*2), 2, 2, INTER_CUBIC);
  //apply region of interest to only view inside of bin
  //Xmin = -120, Xmax = 130, Ymin = -800, Ymax = 250
  Rect roi = Rect(120,45,110,145);
  Mat depthImage = depthImageRaw(roi);
  //Mat xyzImage = xyzImageRaw(roi);
  //minMaxIdx(xyzImage, &min, &max);
  //convertScaleAbs(xyzImage, xyzImage, 255 / max);
  //applyColorMap(xyzImage, xyzImage, COLORMAP_JET);
  //Mat channel[3];
  //split(xyzImage, channel);
  minMaxIdx(depthImage, &min, &max);
  convertScaleAbs(depthImage, depthImage, 255 / max);
  applyColorMap(depthImage, depthImage, COLORMAP_JET);
  //convert to gray scale
  Mat depthImageGray;
  cvtColor(depthImage, depthImageGray, CV_BGR2GRAY);
  //create new mat objects to hold binary data
  Mat depthImageBinary;
  Mat depthImageBinary16S;
  Mat depthImageBinary16SLabeled;
  //create variables for blob detection
  vector<Point2d*> firstPixelvec;
  vector<Point2d*> posVec;
  vector<int> areaVec;
  double x,y;
  //detect blobs going from high to low threshold
  for(int i = 12; i > -1; i--){
    //dynamically apply thresholds
    threshold(depthImageGray, depthImageBinary, 20*i, 1, CV_THRESH_BINARY);
    //use 16S image
    depthImageBinary.convertTo(depthImageBinary16S, CV_16S);
    //call blob detection
    labelBLOBsInfo(depthImageBinary16S,depthImageBinary16SLabeled,firstPixelvec, posVec,areaVec, 200, 10000);
    for(int j = 0; j < firstPixelvec.size(); j++){
       //calculate in meters the x and y position
       x = ((posVec[j]->x * 1.65) - 430)/1000; 
       y = ((posVec[j]->y*1.8) - 250)/1000;
       positions.push_back(Point2d(x,y));
       
       //get depth value (z value?)
       /*uchar val;
       int posy = posVec[j]->y;
       int posx = posVec[j]->x;
       for(int k = 0; k < 3; k++){
        val = channel[k].at<uchar>(posy,posx);
        cout << "Value Channel"<<k<<": " << (double)val << " thresh: " << to_string(20*i) << " at("<<posy<<","<<posx<<")"<< endl;
       }
       circle(depthImageBinary16S, Point(posy,posx), 5, Scalar(10));*/
       show16SImageStretch(depthImageBinary16S, "16s binary");
    }
    //clear vectors to search next threshold
    firstPixelvec.clear(); 
    areaVec.clear(); 
    posVec.clear();
  }
  return positions;
}
