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

	string window_name = "Processed Video";
	namedWindow(window_name, CV_WINDOW_KEEPRATIO); //resizable window;
        Mat image;
	image = imread(argv[1]);	
	imshow("origin",image);
        Mat gray;
       	cvtColor(image,gray,CV_RGB2GRAY);
	vector<string> codes;
	Mat corners;
	Rect roi(0,image.cols/3,image.cols-1,image.rows - image.cols/3);// set the ROI for the image

	Mat imgROI;//
	GaussianBlur(gray(roi),imgROI,Size(3,3),0);
	    // Display the image
	if(showSteps){
		namedWindow("Original Image");
		imshow("Original Image",imgROI);
		imwrite("original.bmp", imgROI);
	}

	   // Canny algorithm
	Mat contours;
	Canny(imgROI,contours,50,250);
	Mat contoursInv;
	threshold(contours,contoursInv,128,255,THRESH_BINARY_INV);
	   // Display Canny image
	if(showSteps){
		namedWindow("Contours");
		imshow("Contours1",contoursInv);
		imwrite("contours.bmp", contoursInv);
	}

	 /* 
		Hough tranform for line detection with feedback
		Increase by 25 for the next frame if we found some lines.  
		This is so we don't miss other lines that may crop up in the next frame
		but at the same time we don't want to start the feed back loop from scratch. 
	*/
/*	std::vector<Vec2f> lines;
	if (houghVote < 1 or lines.size() > 2){ // we lost all lines. reset 
		houghVote = 200; 
	}
	else{ houghVote += 25;} 
	while(lines.size() < 5 && houghVote > 0){
		HoughLines(contours,lines,1,PI/180, houghVote);
		houghVote -= 5;
	}
	imshow("contours111",contours);
	
	std::cout << houghVote << "\n";
	Mat result(imgROI.size(),CV_8U,Scalar(255));
	imgROI.copyTo(result);
	   // Draw the limes
	std::vector<Vec2f>::const_iterator it= lines.begin();
	Mat hough(imgROI.size(),CV_8U,Scalar(0));
	while (it!=lines.end()) {

	    float rho= (*it)[0];   // first element is distance rho
	    float theta= (*it)[1]; // second element is angle theta
		
	    if ( theta > 0.09 && theta < 1.48 || theta < 3.14 && theta > 1.66 ) { // filter to remove vertical and horizontal lines
			
			// point of intersection of the line with first row
		Point pt1(rho/cos(theta),0);        
			// point of intersection of the line with last row
		Point pt2((rho-result.rows*sin(theta))/cos(theta),result.rows);
				// draw a white line
		line( result, pt1, pt2, Scalar(255), 8); 
		line( hough, pt1, pt2, Scalar(255), 8);
	}
			//std::cout << "line: (" << rho << "," << theta << ")\n"; 
		++it;
    }

	    // Display the detected line image
	if(showSteps){
		namedWindow("Detected Lines with Hough");
		imshow("Detected Lines with Hough",result);
		imwrite("hough.bmp", result);
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
	int temp=0;
	Mat leftLane(imgROI.size(),CV_8U,Scalar(0));
	Mat rightLane(imgROI.size(),CV_8U,Scalar(0));
	Mat houghP(imgROI.size(),CV_8U,Scalar(0));
	ld.setShift(0);
	ld.drawLeftLane(leftLane);
	ld.drawRightLane(rightLane);
//	ld.drawDetectedLines(houghP);
	std::cout << "First Hough" << "\n";

	if(showSteps){
		namedWindow("Detected Lines with HoughP");
		imshow("leftLane", leftLane);
		imshow("rightLane", rightLane);
		//imshow("Detected Lines with HoughP", houghP);
//		imwrite("houghP.bmp", houghP);
	}

	   // bitwise AND of the two hough images
	//bitwise_and(houghP,hough,houghP);

		   // Set probabilistic Hough parameters
	ld.setLineLengthAndGap(5,2);
	ld.setMinVote(1);
	ld.setShift(image.cols/3);
	ld.drawDetectedLines(image);
	char key = (char) waitKey(0);
		
}




