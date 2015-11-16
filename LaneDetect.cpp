#include "opencv2/highgui/highgui.hpp"
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <vector>
#include <stdio.h>
#include <raspicam/raspicam_cv.h>
#include "linefinder.h"

#define PI 3.1415926
using namespace cv;
using namespace std;
int main(int argc, char* argv[]) {
        raspicam::RaspiCam_Cv Camera;

        int houghVote = 200;
        bool showSteps =true;

        //VideoCapture capture(argv[1]);
        Camera.set(CV_CAP_PROP_FORMAT, CV_8UC1);
        Camera.set(CV_CAP_PROP_FRAME_WIDTH, 640);
        Camera.set(CV_CAP_PROP_FRAME_HEIGHT, 480);

        //Open camera
        if (!Camera.open()) {cerr<<"Error opening the camera"<<endl;return -1;}

        namedWindow("display",WINDOW_NORMAL);    
        resizeWindow("display",640,480);
        namedWindow("image",WINDOW_NORMAL);    
        resizeWindow("image",640,480);
        Mat image;
        Mat gray;
        LineFinder ld;

        while(1)
        {
                //capture>>image;
                Camera.grab();
                Camera.retrieve(image);
                if(image.empty())
                        break;
//                cvtColor(image,gray,CV_RGB2GRAY);
                Rect roi(0,image.rows/2,image.cols-1,image.rows/2);// set the ROI for the image
                Mat imgROI;//
                GaussianBlur(image(roi),imgROI,Size(3,3),0);
                // Display the image

                // Canny algorithm
                Mat contours;
                Canny(imgROI,contours,75,140);
                imshow("display",contours);
                waitKey(1);
                // Detect lines
                ld.setLineLengthAndGap(60,10);
                ld.setMinVote(4);
                ld.findLines(contours);
                cout<<"after findlines"<<endl;
                //	    std::vector<Vec4i> li= ld.findLines(contours);

                ld.processSide();
                cout<<"after processSide"<<endl;
                ld.laneFilter();
                cout<<"after laneFilter"<<endl;

                ld.calcIntersectP();
                cout<<"after calcIntersectP "<<endl;
                //this intersection has the vanishing point
                //intersection has (x,y) coordinates access it like following
                //intersection.x
                //intersection.y
                Point intersection=ld.getIntersectP();

                ld.setShift(image.rows/2);
                ld.drawLines(image);
                cout<<"After drawLines"<<endl;

                if(showSteps){
                        imshow("image",image);
                        waitKey(1);
                }
                //	    ld.drawDetectedLines(image);
        }
        char key = (char) waitKey(0);

}




