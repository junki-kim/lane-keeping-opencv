#include "opencv2/highgui/highgui.hpp"
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <vector>
#include <stdio.h>
#include <raspicam/raspicam_cv.h>
#include "linefinder.h"

//#include </home/pi/Downloads/wiringPi/wiringPi/wiringPi.h>
//#include </home/pi/Downloads/wiringPi/wiringPi/softPwm.h>
#include <wiringPi.h>
#include <softPwm.h>

#include <pthread.h>
#include <unistd.h>

#define PI 3.1415926

#define LEFT 0
#define RIGHT 1

using namespace cv;
using namespace std;

int x = 320, y = 240;
int start_flag = 0;

void *t_func(void *data) // data -> (x, y)
{
#define NORMAL 100
#define TURN_HIGH 200
#define TURN_LOW 50
#define STD_X 320
#define STD_Y 240
#define OFFSET 50

        while(start_flag)
        {
		printf("in \n");
                if((x < STD_X + OFFSET)&&(x > STD_X - OFFSET))
                {
                        softPwmWrite(LEFT, NORMAL);
                        softPwmWrite(RIGHT, NORMAL);
                }
                else if(x > STD_X + OFFSET)
                {
			printf("right\n");
                        softPwmWrite(LEFT, NORMAL);
                        softPwmWrite(RIGHT, TURN_HIGH);
                }
                else if(x < STD_X - OFFSET)
                {
			printf("left\n");
                        softPwmWrite(LEFT, TURN_HIGH);
                        softPwmWrite(RIGHT, NORMAL);
                }
        }
}
int main(int argc, char* argv[]) {
        raspicam::RaspiCam_Cv Camera;

        int houghVote = 200;
        bool showSteps =true;

        pthread_t p_thread;
        int thd_id;

        if(wiringPiSetup() == -1)
        {
              cout<<"setup error"<<endl;
            return -1;
   }

//       pin setting
     pinMode(LEFT, OUTPUT);
      pinMode(RIGHT, OUTPUT);
       softPwmCreate(LEFT, 50, 200);
      softPwmCreate(RIGHT, 50, 200);

//	system("./tset 0");

        thd_id = pthread_create(&p_thread, NULL, t_func, NULL);

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
#if DEBUG == 1
                cout<<"after findlines"<<endl;
#endif
                //	    std::vector<Vec4i> li= ld.findLines(contours);

                ld.processSide();
#if DEBUG == 1
                cout<<"after processSide"<<endl;
#endif
                ld.laneFilter();
#if DEBUG == 1
                cout<<"after laneFilter"<<endl;
#endif

                ld.calcIntersectP();
#if DEBUG == 1
                cout<<"after calcIntersectP "<<endl;
#endif
                //this intersection has the vanishing point
                //intersection has (x,y) coordinates access it like following
                //intersection.x
                //intersection.y
                Point intersection=ld.getIntersectP();

                start_flag = 1;
               // x = intersection.x;
               // y = intersection.y;

                ld.setShift(image.rows/2);
                ld.drawLines(image);
#if DEBUG == 1
                cout<<"After drawLines"<<endl;
#endif

                if(showSteps){
                        imshow("image",image);
                        waitKey(1);
                }
                //	    ld.drawDetectedLines(image);
        }
        char key = (char) waitKey(0);

}




