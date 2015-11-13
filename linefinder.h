#if !defined LINEF
#define LINEF

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#define PI 3.1415926
using namespace std;
using namespace cv;
class LineFinder {

  private:

	  // original image
	  cv::Mat img;

	  Point intersectP;
	  // vector containing the end points 
	  // of the detected lines
	  std::vector<cv::Vec4i> lines;
	  vector<cv::Vec4i> leftLane;
	  vector<cv::Vec4i> rightLane;  
	  // accumulator resolution parameters
	  double deltaRho;
	  double deltaTheta;

	  // minimum number of votes that a line 
	  // must receive before being considered
	  int minVote;

	  // min length for a line
	  double minLength;

	  // max allowed gap along the line
	  double maxGap;

	  // distance to shift the drawn lines down when using a ROI
	  int shift;

  public:

	  // Default accumulator resolution is 1 pixel by 1 degree
	  // no gap, no mimimum length
	  LineFinder() : deltaRho(1), deltaTheta(PI/180), minVote(10), minLength(0.), maxGap(0.) {}

	  // Set the resolution of the accumulator
	  void setAccResolution(double dRho, double dTheta) {

		  deltaRho= dRho;
		  deltaTheta= dTheta;
	  }

	  // Set the minimum number of votes
	  void setMinVote(int minv) {

		  minVote= minv;
	  }

	  // Set line length and gap
	  void setLineLengthAndGap(double length, double gap) {

		  minLength= length;
		  maxGap= gap;
	  }

	  // set image shift
	  void setShift(int imgShift) {

		  shift = imgShift;
	  }

	  // Apply probabilistic Hough Transform
	  std::vector<cv::Vec4i> findLines(cv::Mat& binary) {

		  lines.clear();
		  cv::HoughLinesP(binary,lines,deltaRho,deltaTheta,minVote, minLength, maxGap);

		  return lines;
	  }

	  // Draw the detected lines on an image
	  void drawDetectedLines(cv::Mat &image, cv::Scalar color=cv::Scalar(255)) {
		
	  // Draw the lines
	  std::vector<cv::Vec4i>::const_iterator it2= lines.begin();
	  while (it2!=lines.end()) {
	      cv::Point pt1((*it2)[0],(*it2)[1]+shift);        
	      cv::Point pt2((*it2)[2],(*it2)[3]+shift);
	      cv::line( image, pt1, pt2, color, 6 );
		std::cout << " HoughP line: ("<< pt1 <<"," << pt2 << ")\n"; 
	      ++it2;	
	  }
	}
	  void drawLeftLane(cv::Mat &image, cv::Scalar color=cv::Scalar(255)) {
		
	  // Draw the lines
	  std::vector<cv::Vec4i>::const_iterator it2= leftLane.begin();
	  while (it2!=leftLane.end()) {
	      cv::Point pt1((*it2)[0],(*it2)[1]+shift);        
	      cv::Point pt2((*it2)[2],(*it2)[3]+shift);
	      cv::line( image, pt1, pt2, color, 3 );
	      ++it2;	
	  }
	  }

	  void drawRightLane(cv::Mat &image, cv::Scalar color=cv::Scalar(255)) {
		
	  // Draw the lines
	  std::vector<cv::Vec4i>::const_iterator it2= rightLane.begin();
	  while (it2!=rightLane.end()) {
	      cv::Point pt1((*it2)[0],(*it2)[1]+shift);        
	      cv::Point pt2((*it2)[2],(*it2)[3]+shift);
	      cv::line( image, pt1, pt2, color, 3 );
	      ++it2;	
	  }
	  }
	  // Eliminates lines that do not have an orientation equals to
	  // the ones specified in the input matrix of orientations
	  // At least the given percentage of pixels on the line must 
	  // be within plus or minus delta of the corresponding orientation
  std::vector<cv::Vec4i> removeLinesOfInconsistentOrientations(
  const cv::Mat &orientations, double percentage, double delta) {
	  std::vector<cv::Vec4i>::iterator it= lines.begin();
	  // check all lines
	  while (it!=lines.end()) {
		  // end points
	  int x1= (*it)[0];
	  int y1= (*it)[1];
	  int x2= (*it)[2];
	  int y2= (*it)[3];
		   
	  // line orientation + 90o to get the parallel line
	  double ori1= atan2(static_cast<double>(y1-y2),static_cast<double>(x1-x2))+PI/2;
	  if (ori1>PI) ori1= ori1-2*PI;

	  double ori2= atan2(static_cast<double>(y2-y1),static_cast<double>(x2-x1))+PI/2;
	  if (ori2>PI) ori2= ori2-2*PI;
		  // for all points on the line
	  cv::LineIterator lit(orientations,cv::Point(x1,y1),cv::Point(x2,y2));
	  int i,count=0;
	  for(i = 0, count=0; i < lit.count; i++, ++lit) { 
	  float ori= *(reinterpret_cast<float *>(*lit));
	  // is line orientation similar to gradient orientation ?
	  if (std::min(fabs(ori-ori1),fabs(ori-ori2))<delta)
		  count++;
	  }
	  double consistency= count/static_cast<double>(i);
	  // set to zero lines of inconsistent orientation
	  if (consistency < percentage) {
		  (*it)[0]=(*it)[1]=(*it)[2]=(*it)[3]=0;
	  }
	  ++it;
	  }

	  return lines;
  }
    void processSide()
    {
	std::vector<cv::Vec4i>::const_iterator it2= lines.begin();
	while (it2!=lines.end()) {
	  cv::Point pt1((*it2)[0],(*it2)[1]+shift);        
	  cv::Point pt2((*it2)[2],(*it2)[3]+shift);
	  double dx=pt2.x-pt1.x;
	  double dy=pt2.y-pt1.y;
	  dx= (dx==0.0)? 1.0:dx;
	  if((dy/dx)>0.0)
	   rightLane.push_back(Vec4i((*it2)[0],(*it2)[1],(*it2)[2],(*it2)[3]));
	  else if((dy/dx)<0.0)
	  leftLane.push_back(Vec4i((*it2)[0],(*it2)[1],(*it2)[2],(*it2)[3]));
	   ++it2;
     }
	
    }
    /*laneFilter finds the lines which has smallest and biggest gradient on right and left lanes respectively.   */
    void laneFilter()
    {
	std::vector<cv::Vec4i>::iterator iter=leftLane.begin();
	double dx=1.0*(*iter)[2]-(*iter)[0];
	double dy=1.0*(*iter)[3]-(*iter)[1];
	dx=(dx==0)? 1.0:dx;
	double gradient=dy/dx;
	iter++;
	cout<<"size : "<<leftLane.size()<<endl;
	while(iter!=leftLane.end())
	{
	    dx=1.0*(*iter)[2]-(*iter)[0];
	    dy=1.0*(*iter)[3]-(*iter)[1];
	    cout<<"("<<(*iter)[0]<<","<<(*iter)[1]<<"), ("<<(*iter)[2]<<","<<(*iter)[3]<<")"<<endl;
	    dx=(dx==0)? 1.0 : dx;
//	    cout<<"be current g : "<<gradient<<"<, dy/dx : "<<dy/dx<<endl;
	    if(gradient<(dy/dx))
	    {
		gradient=dy/dx;
		leftLane.erase(iter-1);
	    }
	    else
		leftLane.erase(iter);
//	    cout<<"af current g : "<<gradient<<"<, dy/dx : "<<dy/dx<<endl;
	    iter++;	
	}
//	std::vector<cv::Vec4i>::iterator iter=leftLane.begin();
	cout<<"left line size : "<<leftLane.size()<<endl;
	iter=rightLane.begin();
	cout<<"size : "<<rightLane.size()<<endl;
	dx=1.0*(*iter)[2]-(*iter)[0];
	dy=1.0*(*iter)[3]-(*iter)[1];
	dx=(dx==0)? 1.0:dx;
	gradient=dy/dx;
	iter++;
	cin>>dx;
	while(iter!=rightLane.end())
	{
	    dx=1.0*(*iter)[2]-(*iter)[0];
	    dy=1.0*(*iter)[3]-(*iter)[1];
	    dx=(dx==0)? 1.0 : dx;
	    cout<<"("<<(*iter)[0]<<","<<(*iter)[1]<<"), ("<<(*iter)[2]<<","<<(*iter)[3]<<")"<<endl;
//	    cout<<"be current g : "<<gradient<<"<, dy/dx : "<<dy/dx<<endl;
	    if(gradient<(dy/dx))
	    {
		gradient=dy/dx;
		rightLane.erase(iter-1);
	    }
	    else
		rightLane.erase(iter);
//	    cout<<"af current g : "<<gradient<<"<, dy/dx : "<<dy/dx<<endl;
	    iter++;	
	}
	cout<<"left lane size: "<<leftLane.size()<<endl;
	cout<<"right lane size: "<<rightLane.size()<<endl;
	cin>>dx;
    }
    Point getIntersectP()
    {
	return intersectP;
    }
    
    void calcIntersectP(){
	int x1=leftLane[0][0];
	int y1=leftLane[0][1];
	int x2=leftLane[0][2];
	int y2=leftLane[0][3];
	int x3=rightLane[0][0];
	int y3=rightLane[0][1];
	int x4=rightLane[0][2];
	int y4=rightLane[0][3];
	double intersecX=(1.0*((x1*y2-y1*x2)*(x3-x4)-(x1-x2)*(x3*y4-y3*x4)))/((x1-x2)*(y3-y4)-(y1-y2)*(x3-x4));
	double intersecY=(1.0*((x1*y2-y1*x2)*(y3-y4)-(y1-y2)*(x3*y4-y3*x4)))/((x1-x2)*(y3-y4)-(y1-y2)*(x3-x4));
	cout<<"intersection ("<<intersecX<<" , "<<intersecY<<")"<<endl;
    }

};


#endif
