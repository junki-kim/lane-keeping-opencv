#include "opencv2/highgui/highgui.hpp"
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <vector>
#include <stdio.h>
#include "linefinder.h"

#define PI 3.1415926

using namespace cv;
using namespace std;
int main(int argc, char* argv[]) {
	int houghVote = 200;
	string arg = argv[1];
	bool showSteps =true;

	VideoCapture capture(argv[1]);
	namedWindow("display",WINDOW_NORMAL);    
	resizeWindow("display",640,480);
	namedWindow("image",WINDOW_NORMAL);    
	resizeWindow("image",640,480);
        Mat image;
 	Mat gray;
	LineFinder ld;

	while(1)
	{
	    capture>>image;
	    if(image.empty())
		break;
	    cvtColor(image,gray,CV_RGB2GRAY);
	    Rect roi(0,image.cols/3,image.cols-1,image.rows - image.cols/3);// set the ROI for the image
	    Mat imgROI;//
	    GaussianBlur(gray(roi),imgROI,Size(3,3),0);
	    // Display the image

	   // Canny algorithm
	    Mat contours;
	    Canny(imgROI,contours,80,100);
	    imshow("display",contours);
	    waitKey(1);
	       // Detect lines
	    ld.setLineLengthAndGap(60,10);
	    ld.setMinVote(4);
	    ld.findLines(contours);
	    std::vector<Vec4i> li= ld.findLines(contours);

	    ld.processSide();
	    ld.laneFilter();
	    
	    ld.calcIntersectP();
	    //this intersection has the vanishing point
	    //intersection has (x,y) coordinates access it like following
	    //intersection.x
	    //intersection.y
	    Point intersection=ld.getIntersectP();

	    ld.setShift(image.cols/3);
	    ld.drawLines(image);
	    
	    if(showSteps){
    		imshow("image",image);
		    waitKey(1);
	    }
//	    ld.drawDetectedLines(image);
	}
	char key = (char) waitKey(0);
		
}




