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
        Mat image;
 	Mat gray;
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
	   /* if(showSteps){
		    imshow("Original Image",imgROI);
	    }*/

	   // Canny algorithm
	    Mat contours;
	    Canny(imgROI,contours,50,250);
	    Mat contoursInv;
	    threshold(contours,contoursInv,128,255,THRESH_BINARY_INV);
	       // Display Canny image
	   /* if(showSteps){
		    namedWindow("Contours");
		    imshow("Contours1",contoursInv);
		    imwrite("contours.bmp", contoursInv);
	    }*/
	       // Create LineFinder instance
	    LineFinder ld;

	       // Set probabilistic Hough parameters
	    ld.setLineLengthAndGap(60,10);
	    ld.setMinVote(4);

	       // Detect lines
	    ld.findLines(contours);
	    std::vector<Vec4i> li= ld.findLines(contours);

	    ld.processSide();
	    ld.laneFilter();
	    
	    ld.calcIntersectP();
//	    Mat leftLane(imgROI.size(),CV_8U,Scalar(0));
//	    Mat rightLane(imgROI.size(),CV_8U,Scalar(0));
//	    Mat houghP(imgROI.size(),CV_8U,Scalar(0));
//	    ld.setShift(0);
//	    ld.drawLeftLane(leftLane);
//	    ld.drawRightLane(rightLane);
//	    std::cout << "First Hough" << "\n";

//	    if(showSteps){
//		    namedWindow("Detected Lines with HoughP");
//		    imshow("leftLane", leftLane);
//		    imshow("rightLane", rightLane);
//	    }

	    // Set probabilistic Hough parameters
	    ld.setLineLengthAndGap(5,2);
	    ld.setMinVote(1);
	    ld.setShift(image.cols/3);
	    ld.drawLines(image);
	    
	    if(showSteps){
	
    		imshow("display",image);
		    waitKey(1);
	    }
//	    ld.drawDetectedLines(image);
	}
	char key = (char) waitKey(0);
		
}




